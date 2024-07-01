import pyrealsense2 as rs
import numpy as np
import cv2
import time
import json
import threading

# Global variables for depth filtering distances
near_clipping_distance = 1.0  # 1 meter
far_clipping_distance = 1.2  # 1.2 meters

# Load the JSON settings
default_json = '''{
}'''

# Function to find a device that supports advanced mode
def find_device_that_supports_advanced_mode():
    DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07", "0B3A", "0B5C", "0B5B"]
    ctx = rs.context()
    devices = ctx.query_devices()
    for dev in devices:
        if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
            if dev.supports(rs.camera_info.name):
                print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
            return dev
    raise Exception("No D400 product line device that supports advanced mode was found")

# Function to print command instructions
def print_instructions():
    print("Commands:")
    print("  q/Esc: Quit the program")
    print("  r: Read and print advanced mode parameters")
    print("  d: Set near and far distances for depth filtering")

# Function to print advanced mode parameters
def print_advanced_mode_parameters():
    print("Depth Control: \n", advnc_mode.get_depth_control())
    print("RSM: \n", advnc_mode.get_rsm())
    print("RAU Support Vector Control: \n", advnc_mode.get_rau_support_vector_control())
    print("Color Control: \n", advnc_mode.get_color_control())
    print("RAU Thresholds Control: \n", advnc_mode.get_rau_thresholds_control())
    print("SLO Color Thresholds Control: \n", advnc_mode.get_slo_color_thresholds_control())
    print("SLO Penalty Control: \n", advnc_mode.get_slo_penalty_control())
    print("HDAD: \n", advnc_mode.get_hdad())
    print("Color Correction: \n", advnc_mode.get_color_correction())
    print("Depth Table: \n", advnc_mode.get_depth_table())
    print("Auto Exposure Control: \n", advnc_mode.get_ae_control())
    print("Census: \n", advnc_mode.get_census())

# Function to set near and far distances for depth filtering
def set_depth_filtering():
    global near_clipping_distance, far_clipping_distance
    try:
        near_clipping_distance = float(input("Enter near clipping distance (meters): "))
        far_clipping_distance = float(input("Enter far clipping distance (meters): "))
        print(f"Depth filtering set: near = {near_clipping_distance}m, far = {far_clipping_distance}m")
    except ValueError:
        print("Invalid input. Please enter numeric values.")

# Function to handle terminal inputs
def input_thread():
    print_instructions()
    while True:
        command = input()
        if command == 'q':
            break
        elif command == 'r':
            print_advanced_mode_parameters()
        elif command == 'd':
            set_depth_filtering()

# Initialize pipeline
pipeline = rs.pipeline()
config = rs.config()
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

# Check if the device has an RGB camera
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

# Enable streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Create an align object
align_to = rs.stream.color
align = rs.align(align_to)

# Find and enable advanced mode on the device
dev = find_device_that_supports_advanced_mode()
advnc_mode = rs.rs400_advanced_mode(dev)
if not advnc_mode.is_enabled():
    advnc_mode.toggle_advanced_mode(True)
    time.sleep(5)  # Wait for the device to re-connect
    dev = find_device_that_supports_advanced_mode()
    advnc_mode = rs.rs400_advanced_mode(dev)

# Load default settings
advnc_mode.load_json(default_json)

# Start the input handling thread
input_thread = threading.Thread(target=input_thread)
input_thread.start()

# Function to estimate continuity in the combined mask
def estimate_continuity(mask):
    # Convert mask to grayscale if it's not already
    if len(mask.shape) > 2:
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)
    
    # Threshold the mask to ensure it's binary
    _, mask = cv2.threshold(mask, 127, 255, cv2.THRESH_BINARY)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Create an empty mask for continuity
    continuity = np.zeros_like(mask)

    # Draw filled contours for continuity
    for contour in contours:
        cv2.drawContours(continuity, [contour], -1, (255), thickness=cv2.FILLED)

    return continuity

# Streaming loop
try:
    while True:
        start_time = time.time()

        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not aligned_depth_frame or not color_frame:
            continue

        # Get depth and color images
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Depth-based filtering (clipping objects within a specific range)
        grey_color = 153
        depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
        bg_removed = np.where((depth_image_3d > (far_clipping_distance / depth_scale)) | (depth_image_3d < (near_clipping_distance / depth_scale)), grey_color, color_image)

        # Apply color thresholding for the RED color (RGB frame processing)
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        lower_red = np.array([170, 120, 70])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv_image, lower_red, upper_red)
        red_mask = mask1 | mask2

        # Combine masks
        combined_mask = cv2.bitwise_and(bg_removed, bg_removed, mask=red_mask)
        continuity_mask = estimate_continuity(combined_mask)

        # Render images
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))

        # Display frame rate
        end_time = time.time()
        fps = 1 / (end_time - start_time)
        cv2.putText(images, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        # Display windows
        cv2.imshow('Align Example', images)
        cv2.imshow('Red Mask', red_mask)
        cv2.imshow('Combined Mask', combined_mask)
        cv2.imshow('Continuity Mask', continuity_mask)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            break

finally:
    pipeline.stop()
    input_thread.join()

