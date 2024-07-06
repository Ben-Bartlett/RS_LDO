"""
RealSense RGB-D Processing Pipeline

1. Import necessary libraries:
   - pyrealsense2 for RealSense camera interaction
   - numpy for numerical operations
   - cv2 for OpenCV image processing
   - open3d for 3D point cloud operations
   - sys for system-specific parameters and functions
   - time for timing operations
   - scipy.interpolate for spline interpolation
   - os.path for path operations

2. Initialize global variables:
   - 'quit', 'pcd', and various configuration parameters for processing and visualization

3. Define utility functions:
   - 'timer': Decorator to measure and print function execution times
   - 'get_intrinsic_matrix': Extract intrinsic camera parameters from a frame
   - 'initialize_realsense_pipeline': Setup RealSense pipeline and configure settings from file

4. Define image processing functions:
   - 'overlay_mask_on_image': Overlay a mask on an image to highlight selected color
   - 'filter_red_colors': Filter out red colors from a frame using LAB color space
   - 'edge_segmentation': Detect edges in a grayscale image using Canny edge detection
   - 'enhance_mask_with_edges': Enhance a mask by combining edge information with color filtering

5. Define point cloud processing functions:
   - 'downsample_point_cloud': Downsample a point cloud using voxel downsampling
   - 'sort_and_average_points': Sort and average points in bins to reduce noise
   - 'fit_spline_to_points': Fit a spline to points using spline interpolation
   - 'shift_points_along_direction': Shift points along their principal axis to refine positioning

6. Define RealSense frame processing functions:
   - 'update_fps_list': Update and calculate average FPS for processing and frame capture
   - 'rgb_processing': Perform RGB image processing to isolate objects of interest
   - 'depth_processing': Process depth data to create a point cloud from masked RGB-D images
   - 'calculate_spline_distance': Calculate Euclidean distances between corresponding points of two splines

7. Define main processing loop:
   - 'process_frames': Continuously capture and process frames from RealSense camera
   - Process RGB and depth data, apply filters, downsample, average points, fit splines, and calculate metrics

8. Main function to initialize RealSense pipeline and start processing frames.

9. Entry point: Start processing frames when executed as main script.

Note: Each function is timed to monitor performance, and key parameters are configured to control processing steps and visualization.

"""

import time
import pyrealsense2 as rs
import numpy as np
import cv2
from memory_profiler import profile
import sys
import os

# Global variable to track total time spent in decorated functions per loop iteration
total_timer_time = 0

# LAB Colour selected from BAG RGB image with seg_tuning_bag.py
selected_color_lab = np.array([77,178,166])
tolerance = 30  # Adjust this tolerance value as needed
canny_threshold1 = 0
canny_threshold2 = 50
kernel_size = 5
kernel = np.ones((kernel_size, kernel_size), np.uint8)
# need to make this the minimum, maybe should remove all canny detection and just go pure LAB for speed
closing_kernel_size = 35

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

@timer
def rgb_processing(frame):
    red_mask = filter_red_colors(frame)
    edge_mask = edge_segmentation(frame)
    enhanced_mask = enhance_mask_with_edges(red_mask, edge_mask)
    overlay_image, object_mask = overlay_mask_on_image(frame, enhanced_mask)
    return overlay_image, object_mask


# Enable this if you want memory use stats at the main loops completion
@profile
def main():
    pipeline = None
    try:
        global total_timer_time
  
        # Select the Bag file
        bag_file, folder_exists = select_bag_file()
        if not folder_exists:
            sys.exit(1)

        # Initialize RealSense pipeline and config
        pipeline = rs.pipeline()
        config = rs.config()
        rs.config.enable_device_from_file(config, bag_file)
        
        # Given I named the bag files with the FPS, we can go and extract it to config the streams 
        # This allows this program to run with either the 30 or 60 hz bagfiles
        fps = extract_fps_before_hz(bag_file)
        if not fps:
            sys.exit(1)

        print("If the program crashed here validate settings of bag")
        config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)
        config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, fps)
        profile = pipeline.start(config)

        frame_count = 0
        desired_frame_count = 60
        loop_count = 0
        desired_loop_count = 60
        last_frame_number = None
        fps_start_time = time.time()
        processing_start_time = time.time()

        try:
            while True:
                # Wait for a coherent pair of frames: depth and color
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()

                if not color_frame:
                    print("bag is missing frame")
                    continue

                # Convert image to numpy array
                color_image = np.asanyarray(color_frame.get_data())

                # Check if the current color frame number is different from the last one
                current_frame_number = frames.get_frame_number()
                if current_frame_number != last_frame_number:
                    frame_count += 1
                    last_frame_number = current_frame_number

                    # Calculate and print FPS every 10 frames
                    if fps == 60 and frame_count == desired_frame_count:
                        end_time = time.time()
                        fps_calced = frame_count / (end_time - fps_start_time)
                        print(f"F FPS: {fps_calced:.2f}")
                        #frame_count = 0
                        fps_start_time = time.time()

                        if fps_calced < 57:
                            print("\nPerformance Warning: Processing time exceeds frame time at 60 FPS\n")             # this runs the output at a similar "print speed" if we half the frame rate and frame count. But this needs to be functionalised maybe rather than duplicate code
                    elif fps == 30 and frame_count == desired_frame_count/2:
                        end_time = time.time()
                        fps_calced = frame_count / (end_time - fps_start_time)
                        print(f"F FPS: {fps_calced:.2f}")
                        #frame_count = 0
                        fps_start_time = time.time()

                        if fps_calced < 27:
                            print("\nPerformance Warning: Processing time exceeds frame time at 30 FPS\n")

                # Reset total timer time for this loop iteration
                total_timer_time = 0


                # Perform RGB processing using CPU-based method
                #overlay_image_cpu, object_mask_cpu = rgb_processing(color_image)




                # Calculate and print total time spent in decorated functions for this loop iteration
                if fps == 60 and frame_count == desired_frame_count:
                    percentage_of_60fps = (total_timer_time / 16.67) * 100
                    print(f"{total_timer_time:.3f} ms / 16.67 ms")
                    # which is {percentage_of_60fps:.2f}% of 16.67 ms (1 frame at 60 fps).")
                    frame_count = 0
                elif fps == 30 and frame_count == desired_frame_count/2:
                    percentage_of_30fps = (total_timer_time / 33.34) * 100
                    print(f"{total_timer_time:.3f} ms / 33.34 ms")
                    # which is {percentage_of_30fps:.2f}% of 33.34 ms (1 frame at 30 fps).")
                    frame_count = 0

              
                # Display the masking results if desired uncomment this
                '''
                cv2.imshow('RGB Masking - CPU', overlay_image_cpu)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                '''


                if fps == 60 and loop_count == desired_loop_count:
                    processing_end_time = time.time()
                    processing_fps_calced = loop_count / (processing_end_time - processing_start_time)
                    print(f"P FPS: {processing_fps_calced:.2f}")
                    processing_start_time = time.time()
                    loop_count = 0
                elif fps == 30 and loop_count == desired_loop_count/2:
                    processing_end_time = time.time()
                    processing_fps_calced = loop_count / (processing_end_time - processing_start_time)
                    print(f"P FPS: {processing_fps_calced:.2f}")
                    processing_start_time = time.time()
                    loop_count = 0
                
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

if __name__ == "__main__":
    main()


'''
NOTES
With just RGB processing, the RGB processing takes longer on average in the 30Hz 
seems to be 4-8ms mostly for 60hz 
seems to be 7-11ms mostly for 30hz

The async nature of this is possibly messing up the code timing estimations somewhat I would say as that async loop is "unknown"

If we turn off RGB processing we see a loop rate of 120Hz and a frame rate of 60Hz.
If we turn on RGB processing we see when the frame rate drops the loop rate also dropped, to a figure below 60Hz. i.e a long iteration of the loop. (higher hz shorter time spent per iteration)


The F FPS we calculate is when we get a new frame.
The P FPS we calculate when we hit the end of the main loop. 
The time we calculate is how long our @timer functions take cumulatively.
Without any processing the main loop runs much much faster than the frames arrive due to the async nature I suppose but without waiting until we get a new frame are we running into situations of time desyncronisation betweeen the main loop and the "frame grabbing"?

Why is the RGB processing taking longer with the 30hz bag files?

'''
