import os
import pyrealsense2 as rs
import numpy as np
import argparse
import cv2
import sys


def list_bag_files():
    bag_files = [f for f in os.listdir("bag_files") if f.endswith('.bag')]
    if not bag_files:
        print("No .bag files found in the 'bag_files' folder.")
        return None
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

def ascii_display(depth_frame):

    # Fixed width for each line (adjust according to your needs)
    LINE_WIDTH = 200

    # Clear the console
    os.system('cls' if os.name == 'nt' else 'clear')

    depth_data = np.asanyarray(depth_frame.get_data())
    coverage = [0] * 64
    for y in range(480):
        for x in range(640):
            dist = depth_frame.get_distance(x, y)
            if 0 < dist < 1:
                coverage[x // 10] += 1
        if y % 20 == 19:
            line = ""
            for c in coverage:
                line += " .:nhBXWW"[c // 25]
            coverage = [0] * 64
            sys.stdout.write(line + "\n")

def read_bag_file(bag_file):
    try:
        pipeline = rs.pipeline()
        config = rs.config()
        rs.config.enable_device_from_file(config, os.path.join("bag_files", bag_file))
        config.enable_stream(rs.stream.depth, rs.format.z16, 30)
        pipeline.start(config)

        while True:
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            ascii_display(depth_frame)
            
    except KeyboardInterrupt:
        print("Ctrl+C detected. Returning to start...")
        return
    finally:
        pipeline.stop()

def main():
    while True:
        selected_file = select_bag_file()
        if selected_file:
            print(f"You have selected: {selected_file}")
            read_bag_file(selected_file)
            print("Finished displaying ASCII art. Returning to start...\n")

if __name__ == "__main__":
    main()

