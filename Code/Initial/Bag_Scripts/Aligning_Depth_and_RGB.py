import os
import pyrealsense2 as rs
import numpy as np
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

def align_depth_and_rgb(frames):
    align = rs.align(rs.stream.color)
    aligned_frames = align.process(frames)

    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    if not aligned_depth_frame or not color_frame:
        return None, None

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # Convert color image from RGB to BGR
    color_image_bgr = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

    return depth_image, color_image_bgr

def remove_background(depth_image, color_image, clipping_distance_in_meters=1):
    clipping_distance = clipping_distance_in_meters / 0.001 #depth_sensor.get_depth_scale() it can be obtained realtime, but here https://support.intelrealsense.com/hc/en-us/community/posts/8177128080915-unable-to-get-depth-scale says 0.001 should be correct for nearly all sensors(i dont know what one we have yet)
    depth_image_3d = np.dstack((depth_image, depth_image, depth_image))
    bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), 153, color_image)

    return bg_removed

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
            depth_image, color_image = align_depth_and_rgb(frames)
            
            if depth_image is not None and color_image is not None:
                bg_removed = remove_background(depth_image, color_image)
                
                cv2.namedWindow('Background Removed', cv2.WINDOW_NORMAL)
                cv2.imshow('Background Removed', bg_removed)
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q') or key == 27:
                    cv2.destroyAllWindows()
                    break
            
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

