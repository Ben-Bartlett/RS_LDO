import os
import pyrealsense2 as rs
import numpy as np
import argparse
import cv2

def list_bag_files():
    bag_files = [f for f in os.listdir("bag_files") if f.endswith('.bag')]
    if not bag_files:
        print("No .bag files found in the 'bag_files' folder.")
    else:
        print("Available .bag files:")
        for i, file in enumerate(bag_files):
            print(f"{i+1}. {file}")
    return bag_files

def select_bag_file():
    bag_files = list_bag_files()
    if bag_files:
        choice = input("Enter the index of the .bag file you want to select: ")
        try:
            index = int(choice) - 1
            if 0 <= index < len(bag_files):
                return bag_files[index]
            else:
                print("Invalid index.")
                return None
        except ValueError:
            print("Invalid input. Please enter a valid index.")
            return None

def select_function():
    print("\nAvailable functions:")
    print("1. Render Depth and RGB Image")
    choice = input("Enter the index of the function you want to select: ")
    return choice

def render_depth_and_rgb(frames):
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        return

    # Convert depth and color frames to numpy arrays
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Apply colormap on depth image
    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    # Convert color image from RGB to BGR
    color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

    # Resize color image to match depth image if necessary
    depth_colormap_dim = depth_colormap.shape
    color_colormap_dim = color_image_bgr.shape
    if depth_colormap_dim != color_colormap_dim:
        resized_color_image = cv2.resize(color_image_bgr, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        images = np.hstack((resized_color_image, depth_colormap))
    else:
        images = np.hstack((color_image_bgr, depth_colormap))

    # Show images
    cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('RealSense', images)
    cv2.waitKey(1)

def read_bag_file(bag_file):
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        rs.config.enable_device_from_file(config, os.path.join("bag_files", bag_file))
        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, rs.format.rgb8, 30)
        pipeline.start(config)

        while True:
            frames = pipeline.wait_for_frames()
            render_depth_and_rgb(frames)
            
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Closing video window.")
        cv2.destroyAllWindows()
        return
    finally:
        pipeline.stop()

def main():
    while True:
        selected_file = select_bag_file()
        if selected_file:
            print(f"You have selected: {selected_file}")
            read_bag_file(selected_file)

if __name__ == "__main__":
    main()

