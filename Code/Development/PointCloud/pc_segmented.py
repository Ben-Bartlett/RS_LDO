import pyrealsense2 as rs
import numpy as np
import cv2
import open3d as o3d
from enum import IntEnum
from datetime import datetime
from os.path import abspath
import sys

sys.path.append(abspath(__file__))

quit = False

# Global variables for selected color in Lab space
selected_color_lab = np.array([74, 176, 163])
tolerance = 40  # Adjust this tolerance value as needed

# Initialize Canny edge thresholds
canny_threshold1 = 10
canny_threshold2 = 50

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

    # Create a config and configure the pipeline to stream color and depth
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()

    # Using preset Default for recording
    depth_sensor.set_option(rs.option.visual_preset, Preset.Default)

    return pipeline, depth_sensor, profile


def enable_advanced_mode(device):
    advanced_mode = rs.rs400_advanced_mode(device)

    # Enable advanced mode if not already enabled
    if not advanced_mode.is_enabled():
        advanced_mode.toggle_advanced_mode(True)

    return advanced_mode


def set_depth_table(advanced_mode, depth_clamp_min=0, depth_clamp_max=10000): #set it in mm (10m)
    # Retrieve and modify the depth table control settings
    depth_table_control = advanced_mode.get_depth_table()
    depth_table_control.depthClampMin = depth_clamp_min
    depth_table_control.depthClampMax = depth_clamp_max

    # Set the updated depth table control settings back to the camera
    advanced_mode.set_depth_table(depth_table_control)

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
    closing_kernel_size = 37
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


def main():
    global quit

    # Initialize the RealSense pipeline
    pipeline, depth_sensor, profile = initialize_realsense_pipeline()

    # Initialize advanced mode for the device
    context = rs.context()
    device = context.query_devices()[0]
    advanced_mode = enable_advanced_mode(device)

    # Set depth clamp values
    set_depth_table(advanced_mode)

    # Create an align object to align depth frames to color frames
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Initialize Open3D visualizer
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(width=600, height=400)

    pcd = o3d.geometry.PointCloud()
    flip_transform = [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]

    # q using GLFW_KEYs
    vis.register_key_callback(81, lambda vis: exit())

    # Streaming loop
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

            # Overlay combined mask on the original image and process depth image
            overlay_image, object_mask = overlay_mask_on_image(frame, enhanced_mask)

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

            temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
            temp.transform(flip_transform)
            pcd.points = temp.points
            pcd.colors = temp.colors

            if frame_count == 0:
                vis.add_geometry(pcd)

            vis.update_geometry(pcd)
            vis.poll_events()
            vis.update_renderer()

            process_time = datetime.now() - dt0
            print("\rFPS: " + str(1 / process_time.total_seconds()), end='')
            frame_count += 1

    finally:
        pipeline.stop()
        vis.destroy_window()


if __name__ == "__main__":
    main()
