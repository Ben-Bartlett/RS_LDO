"""
RealSense RGB-D Processing and Visualization Script

Usage:
------
This script provides functionality to process RGB-D data from Intel RealSense cameras or recorded bag files,
perform object detection based on color and edge detection, generate a 3D point cloud, fit a spline to the
detected object, visualize the processed data in real-time using Open3D, and save the generated point cloud
to a PCD file. The script supports both live camera streaming and playback of recorded bag files.

Functionality Flags:
--------------------
1. `bag_playback`: Boolean flag. Set to `True` to playback from a recorded .bag file, `False` for live camera.
2. `display_rgb_masked`: Boolean flag. Set to `True` to display the masked RGB images during processing.
3. `open3d_vis`: Boolean flag. Set to `True` to visualize the processed point cloud and spline using Open3D.

Dependencies:
-------------
- Python 3.x
- OpenCV (cv2)
- NumPy
- PyRealSense2
- Open3D (o3d)
- SciPy (for spline fitting)
- Memory Profiler (optional, for profiling memory usage)

Global Variables:
-----------------
- `total_timer_time`: Total time spent in decorated functions per loop iteration.
- `vis_open`: Flag indicating if the Open3D visualization window is open.
- `added_pointcloud`: Flag indicating if the point cloud and geometries have been added to the visualization.
- `previous_spline_points`: Array storing the spline points from the previous frame.
- `current_spline_points`: Array storing the spline points from the current frame.
- `selected_color_lab`: LAB color space value used for object detection.
- `tolerance`: Tolerance value used to define the range around `selected_color_lab` for object detection.
- `canny_threshold1`, `canny_threshold2`: Threshold values for Canny edge detection.
- `kernel_size`: Size of the kernel used for morphological operations.
- `closing_kernel_size`: Size of the kernel used for closing operation in image processing.

Global Objects:
---------------
- `pcd`: Open3D PointCloud object to store the 3D point cloud data.
- `downsampled_pcd`: Open3D PointCloud object for the downsampled version of `pcd`.
- `shifted_average_points`: Open3D PointCloud object to visualize the shifted average points along the spline.
- `spline`: Open3D PointCloud object representing the fitted spline.
- `cylinder_mesh`: Open3D TriangleMesh object to represent the cylinders between spline points.

Main Functions:
---------------
1. `main`: Entry point of the script. Initializes camera/stream, processes frames, and visualizes the data.
2. `initialize_camera`: Initializes RealSense pipeline and camera configuration based on user selection.
3. `process_frames`: Waits for coherent depth and color frames, aligns them, and returns aligned frames.
4. `rgb_processing_pipeline`: Executes RGB image processing pipeline: color masking, edge detection, and masking.
5. `depth_processing_pipeline`: Processes depth frame using masked object data and returns Open3D Image and scale.
6. `create_pointcloud`: Generates a 3D point cloud from color and depth data using Open3D functionalities.
7. `pc_processing_pipeline`: Executes point cloud processing pipeline: downsampling, PCA-based sorting, spline fitting.
8. `visualize_pc`: Updates and visualizes the Open3D visualization with the processed data and geometries.
9. `save_callback`: Callback function to save the generated point cloud as a PCD file.
10. `exit_callback`: Callback function to handle window closing event and exit Open3D visualization.

Visualization Functions:
------------------------
1. `intialize_visualization_pc`: Initializes Open3D visualizer and registers key callback functions.
2. `toggle_geometries`, `toggle_geometry`: Functions to toggle visibility of different geometries in Open3D.

Image Processing Functions:
---------------------------
1. `filter_red_colors`: Converts frame to LAB color space and creates a mask for red objects based on `selected_color_lab`.
2. `edge_segmentation`: Performs edge detection on a grayscale image.
3. `enhance_mask_with_edges`: Enhances the red color mask with edge information using morphological operations.
4. `overlay_mask_on_image`: Overlays the processed mask on the original image for visualization.

Point Cloud Processing Functions:
---------------------------------
1. `downsample_point_cloud`: Downsamples the point cloud to reduce the number of points.
2. `sort_and_average_points`: Sorts and averages points using Principal Component Analysis (PCA).
3. `shift_points_along_direction`: Projects points along their direction to simulate a shifted version.
4. `fit_spline_to_points`: Fits a spline to the shifted average points using spline interpolation.
5. `add_cylinders_between_points`: Adds cylinders between spline points for visualization.

Spline Analysis Functions:
---------------------------
1. `calculate_spline_distance`: Calculates the Euclidean distance between corresponding points of two splines.

Note:
-----
This script integrates multiple functionalities to process RGB-D data from RealSense cameras or recorded bag files,
perform object detection, generate 3D point clouds, fit splines, visualize data using Open3D, and save point clouds.
Ensure all dependencies are installed and RealSense camera is connected or bag file path is set correctly before running.
"""


import time
import pyrealsense2 as rs
import numpy as np
import cv2
from memory_profiler import profile
import sys
import os
import open3d as o3d
from datetime import datetime
from scipy.interpolate import splprep, splev


################################################################
#
# EDIT THESE IF YOU WANT TO CHANGE VIS OR TO LIVE CAMERA
#
################################################################

#Functionality Flags to make running file simpler with all functionality
bag_playback = True # set false for live camera
display_rgb_masked = True # set true to see masked RGB (verification mostly)
open3d_vis = True # set True to see the Vis at a 10fps update rate (you can increase the vis rate if you want)
################################################################




# Global variable to track total time spent in decorated functions per loop iteration
total_timer_time = 0
vis_open = True
added_pointcloud = False
previous_spline_points = None
current_spline_points = None

# LAB Colour selected from BAG RGB image with seg_tuning_bag.py
selected_color_lab = np.array([77,178,166])
tolerance = 30  # Adjust this tolerance value as needed
canny_threshold1 = 0
canny_threshold2 = 50
kernel_size = 5
kernel = np.ones((kernel_size, kernel_size), np.uint8)
# need to make this the minimum, maybe should remove all canny detection and just go pure LAB for speed
closing_kernel_size = 35

pcd = o3d.geometry.PointCloud()  # Initialize the global point cloud variable
downsampled_pcd = o3d.geometry.PointCloud() # Initialize the global downsapmpled pointcloud
shifted_average_points = o3d.geometry.PointCloud() # Initialize the global average pointcloud
spline = o3d.geometry.PointCloud() # Initialize the global spline pointcloud
cylinder_mesh = o3d.geometry.TriangleMesh() # Global mesh object to hold cylinder geometries



class ThreadResult:
    def __init__(self):
        self.mask = None

#timer wrapper to "time" functions but we may have better luck timing functions only for a single frame in future to see how fast everything is
def timer(func):
    def wrapper(*args, **kwargs):
        global total_timer_time
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        elapsed_time_ms = (end_time - start_time) * 1000  # Convert to milliseconds
        total_timer_time += elapsed_time_ms
        return result
    return wrapper

# Function to list and select .bag file, should be robust enough to allow the user to run the file,
# in the folder or in the root folder. 
def find_data_folder(start_folder):
    current_folder = os.path.abspath(start_folder)
    while current_folder:
        data_folder = os.path.join(current_folder, "Data")
        if os.path.exists(data_folder):
            # Check if there are .bag files in the Data folder
            bag_files = [f for f in os.listdir(data_folder) if f.endswith('.bag')]
            if bag_files:
                return data_folder
        # Move up one directory
        current_folder = os.path.dirname(current_folder)
    return None

def select_bag_file():
    current_dir = os.getcwd()
    data_folder = find_data_folder(current_dir)
    
    if not data_folder or not os.path.exists(data_folder):
        print(f"No 'Data' folder containing .bag files was found starting from '{current_dir}'. Please run download_bags.sh")
        return None, False
    
    bag_files = [f for f in os.listdir(data_folder) if f.endswith('.bag')]
    if not bag_files:
        print(f"No .bag files found in the 'Data' folder '{data_folder}'. Please run download_bags.sh")
        return None, False
    
    for i, file in enumerate(bag_files):
        print(f"{i}: {file}")
    
    while True:
        try:
            index = int(input(f"Select a .bag file by index (0 to {len(bag_files) - 1}): "))
            if 0 <= index < len(bag_files):
                return os.path.join(data_folder, bag_files[index]), True
            else:
                print("Invalid index. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")


def initialize_camera():

    # Initialize RealSense pipeline and config
    pipeline = rs.pipeline()
    config = rs.config()

    if bag_playback == True:

        bag_file, folder_exists = select_bag_file()
        if not folder_exists:
            sys.exit(1)

        # Given I named the bag files with the FPS, we can go and extract it to config the streams 
        # This allows this program to run with either the 30 or 60 hz bagfiles
        fps = extract_fps_before_hz(bag_file)
        if not fps:
            sys.exit(1)

        rs.config.enable_device_from_file(config, bag_file)
    else: 
        fps_valid = False
        while fps_valid == False:
            try:
                fps = int(input("set the FPS of live camera 30/60: "))
                if fps in [30, 60]:
                    fps_valid = True
                else:
                    print("Invalid fps. 30/60.")
            except ValueError:
                print("Invalid input. Please enter a number.")
        

   
    print("If the program crashed here validate settings of bag or if camera is plugged in")
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, fps)
    
    # Start streaming
    profile = pipeline.start(config)
    depth_sensor = profile.get_device().first_depth_sensor()
    
    return pipeline, depth_sensor, profile, fps

def get_intrinsic_matrix(frame):
    intrinsics = frame.profile.as_video_stream_profile().intrinsics
    return o3d.camera.PinholeCameraIntrinsic(848, 480, intrinsics.fx,
                                             intrinsics.fy, intrinsics.ppx,
                                             intrinsics.ppy)
        

def extract_fps_before_hz(file_path):
    # Get just the filename from the file path
    filename = os.path.basename(file_path)
    
    # Find the index of 'hz' in the filename
    index_hz = filename.find('hz')
    
    if index_hz != -1 and index_hz >= 2:
        # Extract the two characters before 'hz'
        prefix = filename[index_hz - 2:index_hz]
        try:
            # Convert the extracted prefix to an integer
            prefix_int = int(prefix)
            return prefix_int
        except ValueError:
            # Handle cases where the prefix is not a valid integer
            return None
    else:
        # 'hz' not found or not enough characters before 'hz'
        return None
    

def print_fps_and_timer_info(fps, frame_count, desired_frame_count, loop_count, desired_loop_count, fps_start_time, processing_start_time, total_timer_time):
    if fps == 60 and frame_count == desired_frame_count:
        end_time = time.time()
        fps_calced = frame_count / (end_time - fps_start_time)
        print(f"F FPS: {fps_calced:.2f}")
        percentage_of_60fps = (total_timer_time / 16.67) * 100
        print(f"{total_timer_time:.3f} ms / 16.67 ms \n")
        # Reset frame count and start time for next FPS calculation
        frame_count = 0
        fps_start_time = time.time()
        
        if fps_calced < 57:
            print("\nPerformance Warning: Processing time exceeds frame time at 60 FPS\n")
    
    elif fps == 30 and frame_count == desired_frame_count / 2:
        end_time = time.time()
        fps_calced = frame_count / (end_time - fps_start_time)
        print(f"F FPS: {fps_calced:.2f}")
        percentage_of_30fps = (total_timer_time / 33.34) * 100
        print(f"{total_timer_time:.3f} ms / 33.34 ms \n")
        # Reset frame count and start time for next FPS calculation
        frame_count = 0
        fps_start_time = time.time()
        
        if fps_calced < 27:
            print("\nPerformance Warning: Processing time exceeds frame time at 30 FPS\n")
    
    if fps == 60 and loop_count == desired_loop_count:
        processing_end_time = time.time()
        processing_fps_calced = loop_count / (processing_end_time - processing_start_time)
        print(f"P FPS: {processing_fps_calced:.2f}")
        # Reset loop count and start time for next FPS calculation
        loop_count = 0
        processing_start_time = time.time()
    
    elif fps == 30 and loop_count == desired_loop_count / 2:
        processing_end_time = time.time()
        processing_fps_calced = loop_count / (processing_end_time - processing_start_time)
        print(f"P FPS: {processing_fps_calced:.2f}")
        # Reset loop count and start time for next FPS calculation
        loop_count = 0
        processing_start_time = time.time()
    

    return frame_count, loop_count, fps_start_time, processing_start_time

def process_frames(pipeline, align):
    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    return aligned_frames.get_depth_frame(), aligned_frames.get_color_frame()


#RGB processing takes 3 steps here as such 
def filter_red_colors(frame):
    '''
    Convert frame to LAB
    Create a Tolerance Range for the RED value
    Mask the Image to this Range 
    '''
    global selected_color_lab, tolerance, kernel
    lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
    lower_red = np.array([selected_color_lab[0] - tolerance, selected_color_lab[1] - tolerance, selected_color_lab[2] - tolerance])
    upper_red = np.array([selected_color_lab[0] + tolerance, selected_color_lab[1] + tolerance, selected_color_lab[2] + tolerance])
    red_mask = cv2.inRange(lab_frame, lower_red, upper_red)
    return red_mask


def edge_segmentation(frame):
    global canny_threshold1, canny_threshold2
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)
    edges = cv2.Canny(blurred_frame, canny_threshold1, canny_threshold2)
    return edges


def enhance_mask_with_edges(red_mask, edges):
    global kernel, closing_kernel
    dilated_red_mask = cv2.dilate(red_mask, kernel, iterations=1)
    enhanced_mask = cv2.bitwise_and(dilated_red_mask, edges)
    closing_kernel = np.ones((closing_kernel_size, closing_kernel_size), np.uint8)
    enhanced_mask = cv2.morphologyEx(enhanced_mask, cv2.MORPH_CLOSE, closing_kernel)
    return enhanced_mask

def overlay_mask_on_image(image, mask):
    overlay = image.copy()
    object_mask = cv2.merge((mask, mask, mask))
    overlay_masked = cv2.bitwise_and(overlay, object_mask)
    return overlay_masked, object_mask

def rgb_processing_pipeline(color_frame):
    color_data = np.asanyarray(color_frame.get_data())
    overlay_image, object_mask = rgb_processing(color_data)
    if display_rgb_masked:
        cv2.imshow('RGB Masking - CPU', overlay_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            return None, None
    return overlay_image, object_mask

@timer
def rgb_processing(frame):
    # frame passed in as Numpy array
    red_mask = filter_red_colors(frame)
    edge_mask = edge_segmentation(frame)
    enhanced_mask = enhance_mask_with_edges(red_mask, edge_mask)
    overlay_image, object_mask = overlay_mask_on_image(frame, enhanced_mask)
    return overlay_image, object_mask


def depth_processing_pipeline(depth_frame, object_mask, depth_sensor):
    depth_data = np.asanyarray(depth_frame.get_data())
    depth_data[np.where(object_mask[:, :, 0] == 0)] = 0
    return o3d.geometry.Image(depth_data), depth_sensor.get_depth_scale()


def create_pointcloud(depth_image, overlay_image, depth_scale, intrinsic):
    global pcd

    color_temp = cv2.cvtColor(overlay_image, cv2.COLOR_RGB2BGR)
    color_image = o3d.geometry.Image(color_temp)

    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color_image,
        depth_image,
        depth_scale=1.0 / depth_scale,
        convert_rgb_to_intensity=False
    )

    # the use of temp and global pcd may be unccessary but it was the least error prone way to
    # visualize and use keycallbacks

    temp = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)
    temp.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    
    pcd.points = temp.points
    pcd.colors = temp.colors

    return pcd

def intialize_visualization_pc():
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(width=600, height=400)
    
    # Register key callbacks
    vis.register_key_callback(81, exit_callback)  # 'q' key
    vis.register_key_callback(83, save_callback)  # 's' key
    # this is the pain of using the library and its keycallbacks, Might be a cleaner way to do this but sure
    vis.register_key_callback(77, lambda vis: toggle_geometries(vis, 77))  # 'm' key
    vis.register_key_callback(68, lambda vis: toggle_geometries(vis, 68))  # 'd' key
    vis.register_key_callback(65, lambda vis: toggle_geometries(vis, 65))  # 'p' key
    vis.register_key_callback(80, lambda vis: toggle_geometries(vis, 80))  # 'a' key
    vis.register_key_callback(67, lambda vis: toggle_geometries(vis, 67))  # 'c' key

    return vis

def visualize_pc(vis, frame_count, fps):
    global added_pointcloud, downsampled_pcd, shifted_average_points, spline, cylinder_mesh
    if added_pointcloud == False:
        # to add the object once at start
        vis.add_geometry(pcd)
        vis.add_geometry(downsampled_pcd)
        vis.add_geometry(shifted_average_points)
        vis.add_geometry(spline)
        vis.add_geometry(cylinder_mesh)
        added_pointcloud = True
        print("added poincloud")


    elif frame_count % 6 == 0 and fps == 60: 
        # only update pointcloud every 10 frames but it is still calculating everyframe
        vis.update_geometry(pcd)
        vis.update_geometry(downsampled_pcd)
        vis.update_geometry(shifted_average_points)
        vis.update_geometry(spline) 
        vis.update_geometry(cylinder_mesh) 

        
    elif frame_count % 3 == 0 and fps == 30: 
        vis.update_geometry(pcd)
        vis.update_geometry(downsampled_pcd)
        vis.update_geometry(shifted_average_points)
        vis.update_geometry(spline) 
        vis.update_geometry(cylinder_mesh) 
        
    vis.poll_events()    
    vis.update_renderer()



def exit_callback(vis):
    global quit, open3d_vis, vis_open
    quit = True
    vis.destroy_window()
    open3d_vis = False
    vis_open = False


def save_callback(vis):
    global pcd
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"point_cloud_{timestamp}.pcd"
    o3d.io.write_point_cloud(filename, pcd)
    print(f"Point cloud saved as {filename}")


# Toggle function for all geometries
def toggle_geometries(vis, key):
    global pcd, downsampled_pcd, shifted_average_points, spline, cylinder_mesh

    if key == 77:  # 'm' key
        toggle_geometry(vis, pcd)
    elif key == 68:  # 'd' key
        toggle_geometry(vis, downsampled_pcd)
    elif key == 65:  # 'p' key
        toggle_geometry(vis, spline)
    elif key == 80:  # 'a' key
        toggle_geometry(vis,shifted_average_points )
    elif key == 67:  # 'c' key
        toggle_geometry(vis, cylinder_mesh)    

# Function to toggle geometry visibility
def toggle_geometry(vis, geometry):
    if geometry is None:
        return False

    bool_remove = vis.remove_geometry(geometry)

    if not bool_remove:
        vis.add_geometry(geometry)
    
    return bool_remove
        


def downsample_point_cloud(pcd, voxel_size):
    global downsampled_pcd
    # Remove noise from the point cloud
    # But this is very slow and we already have a "good" pointcloud
    #cl, ind = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    #pcd = pcd.select_by_index(ind)

    # Downsample
    downsampled_pcd_temp = pcd.voxel_down_sample(voxel_size=voxel_size)

    # again the temp just allows the real time viewer to update correctly
    # otherwise the pcd doesnt update/move/clear as expected as we are like
    # creating a new object nearly rather than seeing a change to the data in the pcd
    # to trigger the poll and update. 
    downsampled_pcd.points = downsampled_pcd_temp.points
    downsampled_pcd.colors = downsampled_pcd_temp.colors
 
    return downsampled_pcd    


def sort_and_average_points(points, bin_size):
    # Compute centroid
    centroid = np.mean(points, axis=0)

    # Compute covariance matrix
    covariance_matrix = np.cov(points.T)

    # Compute eigenvalues and eigenvectors of covariance matrix
    eigen_values, eigen_vectors = np.linalg.eigh(covariance_matrix)

    # Sort eigenvalues and eigenvectors in descending order
    sort_indices = np.argsort(eigen_values)[::-1]
    eigen_values = eigen_values[sort_indices]
    eigen_vectors = eigen_vectors[:, sort_indices]

    # Use the first three eigenvectors as principal axes
    principal_axes = eigen_vectors[:, :3]

    # Sort points based on the first principal axis
    sorted_indices = np.argsort(np.dot(points - centroid, principal_axes[:, 0]))

    # Bin size calculation
    num_bins = len(points) // bin_size

    # Initialize lists for binned points
    averaged_points = []

    # Bin points and colors
    for i in range(num_bins):
        bin_start = i * bin_size
        bin_end = (i + 1) * bin_size if i < num_bins - 1 else len(points)

        # Calculate average point and color in the bin
        avg_point = np.mean(points[sorted_indices[bin_start:bin_end]], axis=0)
        averaged_points.append(avg_point)

    
    return np.array(averaged_points)


def shift_points_along_direction(spline_points, radius):
    shifted_points = []
    
    for point in spline_points:
        direction = point - np.array([0, 0, 0])  # Assuming origin is [0, 0, 0]
        direction_unit = direction / np.linalg.norm(direction)
        shifted_point = point + direction_unit * radius
        shifted_points.append(shifted_point)
    
    return np.array(shifted_points)


def add_cylinders_between_points(vis, spline_points):
    global cylinder_mesh

    radius = 0.005  # Adjust as needed
    resolution = 10  # Number of faces around the cylinder

    # Reset mesh to clear previous geometries
    cylinder_mesh.clear()

    # Take every tenth point from spline_points
    number_of_points = int(len(spline_points)/10)
    points_to_use = spline_points[::number_of_points]

    # Split the points into two halves
    mid_index = len(points_to_use) // 2
    left_half_points = points_to_use[:mid_index + 1]
    right_half_points = points_to_use[mid_index:]

    # Helper function to create cylinder meshes
    def create_cylinder_mesh(points):
        mesh = o3d.geometry.TriangleMesh()
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i + 1]
            direction = end_point - start_point
            length = np.linalg.norm(direction)

            mid_point = (start_point + end_point) / 2
            origin_to_mid_point = mid_point - np.array([0, 0, 0])
            origin_to_mid_point_unit = origin_to_mid_point / np.linalg.norm(origin_to_mid_point)
            offset_mid_point = mid_point + origin_to_mid_point_unit * radius

            cylinder = create_cylinder_segment(direction, length, radius, resolution)
            cylinder.translate(offset_mid_point)
            mesh += cylinder  # Add cylinder mesh to the local mesh object

        return mesh

    # Create cylinder meshes for left and right halves
    # This may be adding noting really. Just for the fact we are only genertaing some segments
    # For visual purposes, i would not worry if gaps are appernt as we are not
    # creating a cylinder for every section of the spline and i am not taking a nice even number of points

    left_half_mesh = create_cylinder_mesh(left_half_points)
    right_half_mesh = create_cylinder_mesh(right_half_points)

    # Add left and right half meshes to the global cylinders mesh, dont really care if they are actually left or right
    cylinder_mesh += left_half_mesh
    cylinder_mesh += right_half_mesh

    return cylinder_mesh

def create_cylinder_segment(direction, length, radius, resolution):
    # Create a cylinder segment between two points
    cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length, resolution=resolution)
    
    # Align the cylinder with the direction vector
    z_axis = np.array([0, 0, 1])
    axis = np.cross(z_axis, direction)
    angle = np.arccos(np.dot(z_axis, direction / length))
    
    if np.linalg.norm(axis) > 0:
        axis /= np.linalg.norm(axis)
        R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
        cylinder.rotate(R, center=(0, 0, 0))
    
    return cylinder

def fit_spline_to_points(points, num_points, smoothing):
    # Fit the spline using the averaged points
    tck, u = splprep(points.T, u=None, s=smoothing, per=0)
    u_fine = np.linspace(u.min(), u.max(), num_points)
    x_fine, y_fine, z_fine = splev(u_fine, tck)

    return np.vstack((x_fine, y_fine, z_fine)).T



def calculate_spline_distance():
    """
    Calculate the Euclidean distance between corresponding points of spline2 and spline1.
    
    Args:
    - spline1 (numpy.ndarray): Array of points representing the first spline.
    - spline2 (numpy.ndarray): Array of points representing the second spline.
    
    Returns:
    - numpy.ndarray: Array of distances between corresponding points of spline2 and spline1.
    """
    global previous_spline_points, current_spline_points

    def calculate_spline_length(spline):
        return np.sum(np.linalg.norm(np.diff(spline, axis=0), axis=1))
    
    def are_splines_same_length(spline1, spline2, length_tolerance=1e-3):
        length1 = calculate_spline_length(spline1)
        length2 = calculate_spline_length(spline2)
        return abs(length1 - length2) <= length_tolerance
    

    # Check if splines have approximately the same length
    if not are_splines_same_length(previous_spline_points, current_spline_points):
        print("Splines are of different lengths, cannot compute distances.")
        return None
    else:
        print("Δ P(α) = [X2(α) - X1(α); Y2(α) - Y1(α); Z2(α) - Z1(α)]")
        # Compute the distances between corresponding points
        delta_x = current_spline_points[:, 0] - previous_spline_points[:, 0]
        delta_y = current_spline_points[:, 1] - previous_spline_points[:, 1]
        delta_z = current_spline_points[:, 2] - previous_spline_points[:, 2]
        # Compute the Euclidean distances between corresponding points
        distances = np.linalg.norm(current_spline_points - previous_spline_points, axis=1)
    
    return distances, delta_x, delta_y, delta_z

def pc_processing_pipeline(pcd):
    # Downsample the point cloud to reduce the number of points
    global downsampled_pcd, shifted_average_points, spline, previous_spline_points,current_spline_points
    
    #downsampled_pcd = downsample_point_cloud(pcd, voxel_size=0.00075)
    downsampled_pcd = downsample_point_cloud(pcd, voxel_size=0.00075)
    
    
    points = np.asarray(downsampled_pcd.points)

    if points.shape[0] <= 3:
        print("Not enough points to fit a spline.")
        return previous_spline_points


    # Sort and average points using PCA which is robust to the direction of points
    averaged_points = sort_and_average_points(points, bin_size=50)

    # Project all points a radius along the direction vector from the origin to the point
    # Shift average points a radius away from the camera
    shifted_points = shift_points_along_direction(averaged_points, 0.005)

        # Fit a spline to the averaged points, in this case 1001 points on the spline
    # 1001 instead of 1000 is just so there are 1000 "sections" calculated if we are talking about
    # a LDO 
    # (i.e α 0->1 with a resoltiion of 0.001) 
    current_spline_points = fit_spline_to_points(shifted_points, num_points=1001, smoothing=0.5)


    if open3d_vis:
        
        # we will assign the points to the 2 pcds we created here for the:
        # averaged points 
        shifted_average_points.points = o3d.utility.Vector3dVector(shifted_points)
        num_points = shifted_points.shape[0]
        color = [0 , 1, 0] # Green
        colors_np = np.tile(color, (num_points, 1))
        shifted_average_points.colors = o3d.utility.Vector3dVector(colors_np)

        # and the spline
        spline.points = o3d.utility.Vector3dVector(current_spline_points)
        num_points = current_spline_points.shape[0]
        color = [0 , 0, 1] # Blue
        colors_np = np.tile(color, (num_points, 1))
        spline.colors = o3d.utility.Vector3dVector(colors_np)
    
    
    spline_length = np.sum(np.linalg.norm(np.diff(current_spline_points, axis=0), axis=1))
    print(f"Spline Length: {spline_length:.2f} m")


    if previous_spline_points is not None:
        result = calculate_spline_distance()
        # we do this check as if the splines are different lengths we do not calculate a result
        # returning none (could be more graceful i guess but sure, i am squeezing a lot of options
        # into this file with alot of code segregation into functions)
        if result is not None:
            distance_change, delta_x, delta_y, delta_z = result
            ###
            # This is our Δ P(α) = [X2(α) - X1(α); Y2(α) - Y1(α); Z2(α) - Z1(α)]
            ###
    
    return previous_spline_points, current_spline_points


# Enable this if you want memory use stats at the main loops completion
#@profile
def main():
    pipeline = None
    vis = None

    try:
        
        # setup pipeline for live or bag file, set flag at top to change option
        pipeline, depth_sensor, profile, fps = initialize_camera()
        # Create an align object to align depth to color
        align = rs.align(rs.stream.color)

        # initialize some varibales for fps counting
        frame_count = loop_count = 0
        desired_frame_count = desired_loop_count = 60 
        last_frame_number = None
        fps_start_time = processing_start_time  = time.time()

        # Initialize Open3D visualizer
        if open3d_vis == True:
            vis = intialize_visualization_pc()
            

        try:
            while True:
                # Wait for a coherent pair of frames: depth and color, and align them
                aligned_depth_frame, aligned_color_frame = process_frames(pipeline, align)

                # Validate that both frames are valid
                if not aligned_depth_frame or not aligned_color_frame:
                    print("bagfile missing frame or live camera missed frame")
                    continue

                # Check if the current color frame number is different from the last one
                current_frame_number = aligned_color_frame.get_frame_number()
                if current_frame_number != last_frame_number:
                    frame_count += 1
                    last_frame_number = current_frame_number

                #################################################################################
                    

                '''
                RGB Processing
                '''
                # Perform RGB processing using CPU-based method
                overlay_image, object_mask = rgb_processing_pipeline(aligned_color_frame)
                if overlay_image is None:
                    break

                
                '''
                Depth Processing
                '''
                # Perfrom Depth Processing 
                depth_image, depth_scale = depth_processing_pipeline(aligned_depth_frame, object_mask, depth_sensor)



                '''
                Pointcloud creation
                '''
                intrinsic = get_intrinsic_matrix(aligned_color_frame)

                global pcd
                pcd = create_pointcloud(depth_image, overlay_image, depth_scale, intrinsic)


                '''
                Pointcloud Processing to get spline
                '''         
                global previous_spline_points, current_spline_points
                previous_spline_points, current_spline_points = pc_processing_pipeline(pcd)
                previous_spline_points = current_spline_points
                
                

                #################################################################################              
                if open3d_vis == True:
                    # Update or add cylinders between spline points
                    global cylinder_mesh
                    cylinder_mesh = add_cylinders_between_points(vis, current_spline_points)
                    visualize_pc(vis, frame_count, fps)

                # closes loop if vis is closed with q same as the opencv window
                global vis_open
                if not vis_open:
                    break    

                

                # Reset total timer time for this loop iteration
                global total_timer_time
                total_timer_time = 0

                # Print FPS and timer information
                frame_count, loop_count, fps_start_time, processing_start_time = print_fps_and_timer_info(
                    fps, frame_count, desired_frame_count, loop_count, desired_loop_count, fps_start_time, processing_start_time, total_timer_time
                )
                
                loop_count += 1

        except KeyboardInterrupt:
            print("Process interrupted by user")
        except Exception as e:
            print(f"Error during processing: {e}")

    except Exception as e:
        print(f"Initialization error: {e}")
    finally:
        if pipeline:
            pipeline.stop()
            cv2.destroyAllWindows()
        if open3d_vis:
            vis.destroy_window()

if __name__ == "__main__":
    main()


