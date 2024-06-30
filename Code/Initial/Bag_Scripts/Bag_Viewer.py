import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import os

# Function to list and select .bag file
def select_bag_file():
    bag_folder = 'bag_files'
    if not os.path.exists(bag_folder):
        raise FileNotFoundError(f"The folder '{bag_folder}' does not exist. Please download the sample bag files")
    
    bag_files = [f for f in os.listdir(bag_folder) if f.endswith('.bag')]
    if not bag_files:
        raise FileNotFoundError("No .bag files found in the 'bag_files' folder.")
    
    for i, file in enumerate(bag_files):
        print(f"{i}: {file}")
    
    while True:
        try:
            index = int(input(f"Select a .bag file by index (0 to {len(bag_files) - 1}): "))
            if 0 <= index < len(bag_files):
                return os.path.join(bag_folder, bag_files[index])
            else:
                print("Invalid index. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

# Main function to process and display frames
def main():
    try:
        bag_file = select_bag_file()

        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device_from_file(bag_file)
        profile = pipe.start(cfg)

        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        align_to = rs.stream.color
        align = rs.align(align_to)

        for i in range(100): # Initialization frames
            frames = pipe.wait_for_frames()

        aligned_frames = align.process(frames)
        intrinsics = aligned_frames.get_depth_frame().profile.as_video_stream_profile().intrinsics

        while True:
            frames = pipe.wait_for_frames()
            aligned_frames = align.process(frames)
            
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()

            # Convert color frame to BGR format
            color_image = np.asanyarray(color_frame.get_data())
            color_image_bgr = cv.cvtColor(color_image, cv.COLOR_RGB2BGR)

            cv.imshow('Color frame', color_image_bgr)
            cv.imshow("Depth frame", np.asanyarray(rs.colorizer().colorize(depth_frame).get_data()))
            if cv.waitKey(1) == 27: # Exit on pressing 'ESC'
                break

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        cv.destroyAllWindows()
        pipe.stop()

if __name__ == "__main__":
    main()

