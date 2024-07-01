import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from enum import IntEnum
from datetime import datetime
from os.path import abspath
import sys
from scipy.interpolate import splprep, splev
from scipy.signal import savgol_filter

sys.path.append(abspath(__file__))

quit = False
pcd = o3d.geometry.PointCloud()  # Initialize the global point cloud variable
spline_lineset = o3d.geometry.LineSet()  # Initialize the global spline lineset
cylinders = []  # List to hold cylinder geometries

# Global variables for selected color in Lab space
selected_color_lab = np.array([67, 170, 158])
tolerance = 60  # Adjust this tolerance value as needed

# Initialize Canny edge thresholds
canny_threshold1 = 0
canny_threshold2 = 25

# Define kernel for morphological operations
kernel_size = 5
kernel = np.ones((kernel_size, kernel_size), np.uint8)


class Preset(IntEnum):
    Custom = 0
    Default = 1
    Hand = 2
    HighAccuracy = 3
    HighDensity = 4
    MediumDensity = 5


def get_intrinsic_matrix(frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    return o3d.camera.PinholeCameraIntrinsic(640, 480, intrinsics.fx,
                                             intrinsics.fy, intrinsics.ppx,
                                             intrinsics.ppy)


def initialize_realsense_pipeline():
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream color and depth from the bag file
    config = rs.config()
    print("adding bagfile stream")
    rs.config.enable_device_from_file(config, "bags/bag1.bag")  # Replace with your bag file path
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Set desired visual preset (HighAccuracy in this case)
    # depth_sensor.set_option(rs.option.visual_preset, Preset.HighAccuracy)

    # Read the visual preset option
    visual_preset = depth_sensor.get_option(rs.option.visual_preset)
    print(f"Visual preset used in the bag file: {visual_preset}")

    return pipeline, depth_sensor, profile


def set_depth_table(depth_sensor, depth_clamp_min=0, depth_clamp_max=400):
    # Retrieve and modify the depth table control settings
    depth_table_control = depth_sensor.get_depth_table()
    depth_table_control.depthClampMin = depth_clamp_min
    depth_table_control.depthClampMax = depth_clamp_max

    # Set the updated depth table control settings back to the camera
    depth_sensor.set_depth_table(depth_table_control)

    print("depthClampMin has been set to:", depth_table_control.depthClampMin)
    print("depthClampMax has been set to:", depth_table_control.depthClampMax)


def overlay_mask_on_image(image, mask):
    # Create a copy of the original image
    overlay = image.copy()

    # Generate a binary mask for the object
    object_mask = cv2.merge((mask, mask, mask))

    # Set the unmasked area to black in the original image
    overlay_masked = cv2.bitwise_and(overlay, object_mask)

    return overlay_masked, object_mask


def filter_red_colors(frame):
    global selected_color_lab, tolerance, kernel

    # Convert frame to Lab
    lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)

    # Define range around selected color in Lab space
    lower_red = np.array([selected_color_lab[0] - tolerance, selected_color_lab[1] - 20, selected_color_lab[2] - 20])
    upper_red = np.array([selected_color_lab[0] + tolerance, selected_color_lab[1] + 20, selected_color_lab[2] + 20])

    # Create mask for red colors in Lab space
    red_mask = cv2.inRange(lab_frame, lower_red, upper_red)

    return red_mask


def enhance_mask_with_edges(red_mask, edges):
    global kernel

    # Dilate the red mask until it reaches the edges
    dilated_red_mask = cv2.dilate(red_mask, kernel, iterations=1)

    # Combine the dilated red mask with the edges using bitwise AND
    enhanced_mask = cv2.bitwise_and(dilated_red_mask, edges)

    # Apply morphological closing to refine the mask
    closing_kernel_size = 17
    closing_kernel = np.ones((closing_kernel_size, closing_kernel_size), np.uint8)
    enhanced_mask = cv2.morphologyEx(enhanced_mask, cv2.MORPH_CLOSE, closing_kernel)

    return enhanced_mask


def edge_segmentation(frame):
    global canny_threshold1, canny_threshold2

    # Convert frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blurring to reduce noise
    blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred_frame, canny_threshold1, canny_threshold2)

    thickening_kernel_size = 3
    thickening_kernel = np.ones((thickening_kernel_size, thickening_kernel_size), np.uint8)

    # Dilate to make the edges slightly thicker
    dilated_edges = cv2.dilate(edges, thickening_kernel, iterations=1)

    return dilated_edges


def downsample_point_cloud(pcd, voxel_size=0.01):
    downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return downsampled_pcd


def sort_and_average_points(points, bin_size=10):
    # Sort points by their x-coordinates
    points = points[np.argsort(points[:, 0])]

    # Bin points and compute the average of each bin
    averaged_points = []
    for i in range(0, len(points), bin_size):
        bin_points = points[i:i + bin_size]
        averaged_point = np.mean(bin_points, axis=0)
        averaged_points.append(averaged_point)

    return np.array(averaged_points)


def fit_spline_to_points(points, num_points=25):
    # Fit the spline using the averaged points
    tck, u = splprep([points[:, 0], points[:, 1], points[:, 2]], s=1)
    u_fine = np.linspace(0, 1, num_points)
    x_fine, y_fine, z_fine = splev(u_fine, tck)

    return np.vstack((x_fine, y_fine, z_fine)).T


def toggle_point_cloud(vis, pcd):
    bool_remove = vis.remove_geometry(pcd)

    if bool_remove == False:
        vis.add_geometry(pcd)


def exit_callback(vis):
    global quit
    quit = True
    vis.destroy_window()


def save_callback(vis):
    global pcd
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"point_cloud_{timestamp}.pcd"
    o3d.io.write_point_cloud(filename, pcd)
    print(f"Point cloud saved as {filename}")


# Register 'm' key to toggle visibility of the point cloud
def toggle_callback(vis):
    toggle_point_cloud(vis, pcd)


def add_cylinders_between_points(vis, spline_points, tck):
    global cylinders
    radius = 0.005  # Adjust as needed
    resolution = 10 # decagon representation of circle
    split = 1 # height not split less faces

    # Remove existing cylinders
    for cylinder in cylinders:
        vis.remove_geometry(cylinder)
    cylinders.clear()

    # Add new cylinders between spline points
    for i in range(len(spline_points) - 1):
        start_point = spline_points[i]
        end_point = spline_points[i + 1]

        direction = end_point - start_point
        length = np.linalg.norm(direction)

        mid_point = (start_point + end_point) / 2
        origin_to_mid_point = mid_point - np.array([0, 0, 0])
        origin_to_mid_point_unit = origin_to_mid_point / np.linalg.norm(origin_to_mid_point)
        offset_mid_point = mid_point + origin_to_mid_point_unit * radius

        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length, resolution=resolution, split=split)

        z_axis = np.array([0, 0, 1])
        axis = np.cross(z_axis, direction)
        angle = np.arccos(np.dot(z_axis, direction / length))

        if np.linalg.norm(axis) > 0:
            axis /= np.linalg.norm(axis)
            R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
            cylinder.rotate(R, center=(0, 0, 0))

        cylinder.translate(offset_mid_point)
        vis.add_geometry(cylinder)
        cylinders.append(cylinder)
    
    # Set the initial view parameters
    #view_ctl = vis.get_view_control()
    #view_ctl.set_front([-0.63246362951347912, 0.39482011410282669, 0.66641341136149679])
    #view_ctl.set_lookat([-0.0096679043630299253, -0.017195244617371885, -0.28212898813919807])
    #view_ctl.set_up([0.16483440358395104, 0.90923979408575517, -0.38224680017760365])
    #view_ctl.set_zoom(0.69999999999999996)

    
def main():
    global quit, pcd
    print("Initializing the Script")

    # Initialize the RealSense pipeline for bag file playback
    pipeline, depth_sensor, profile = initialize_realsense_pipeline()

    # Create an align object to align depth frames to color frames
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Initialize Open3D visualizer
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(width=600, height=400)

    pcd = o3d.geometry.PointCloud()
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

    # Register key callbacks
    vis.register_key_callback(81, exit_callback)  # 'q' key
    vis.register_key_callback(83, save_callback)  # 's' key
    vis.register_key_callback(77, toggle_callback)  # 'm' key


    frame_count = 0
    try:
        while not quit:
            dt0 = datetime.now()

            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            intrinsic = get_intrinsic_matrix(color_frame)

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            # Convert to numpy array
            frame = np.asanyarray(color_frame.get_data())

            # Perform segmentation
            red_mask = filter_red_colors(frame)
            edge_mask = edge_segmentation(frame)
            enhanced_mask = enhance_mask_with_edges(red_mask, edge_mask)

            # Overlay combined mask on the original image
            overlay_image, object_mask = overlay_mask_on_image(frame, enhanced_mask)

            # Get depth data
            depth_data = np.asanyarray(aligned_depth_frame.get_data())

            # Set depth values to 0 for pixels outside the mask
            depth_data[np.where(object_mask[:, :, 0] == 0)] = 0

            # Create Open3D Image from modified depth data
            depth_image = o3d.geometry.Image(depth_data)
            color_temp = overlay_image
            color_temp = cv2.cvtColor(color_temp, cv2.COLOR_RGB2BGR)
            color_image = o3d.geometry.Image(color_temp)

            # Create RGBD image from color and depth images
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                color_image,
                depth_image,
                depth_scale=1.0 / depth_sensor.get_depth_scale(),
                convert_rgb_to_intensity=False
            )

            # Create point cloud from RGBD image
            temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
            temp.transform(flip_transform)
            pcd.points = temp.points
            pcd.colors = temp.colors

            # Downsample the point cloud to reduce the number of points
            downsampled_pcd = downsample_point_cloud(temp, voxel_size=0.004)
            points = np.asarray(downsampled_pcd.points)

            if points.shape[0] <= 3:
                print("Not enough points to fit a spline.")
                continue

            # Sort and average points
            averaged_points = sort_and_average_points(points, bin_size=30)

            # Fit a spline to the averaged points
            spline_points = fit_spline_to_points(averaged_points)

            # Create a line set for the spline
            lines = [[i, i + 1] for i in range(len(spline_points) - 1)]
            spline_lineset.points = o3d.utility.Vector3dVector(spline_points)
            spline_lineset.lines = o3d.utility.Vector2iVector(lines)
            spline_lineset.paint_uniform_color([0, 0, 1])  # Blue color for the spline

            # Update or add cylinders between spline points
            add_cylinders_between_points(vis, spline_points, None)

            if frame_count == 0:
                vis.add_geometry(pcd)
                vis.add_geometry(spline_lineset)
            else:
                vis.update_geometry(pcd)
                vis.update_geometry(spline_lineset)

            vis.poll_events()
            vis.update_renderer()

            process_time = datetime.now() - dt0
            print("\rFPS: " + str(1 / process_time.total_seconds()), end='')
            frame_count += 1
            cv2.waitKey(1)  # Wait for a small amount of time to display the frame

    finally:
        pipeline.stop()
        vis.destroy_window()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

