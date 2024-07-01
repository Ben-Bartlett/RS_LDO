import open3d as o3d
import os
import numpy as np


def list_pcd_files():
    # List all .pcd files in the './bags' directory
    files = [os.path.join('./bags', f) for f in os.listdir('./bags') if f.endswith('.pcd')]
    return files

def select_pcd_file(files):
    # Prompt user to select a file by index
    for i, file in enumerate(files):
        print(f"{i}: {file}")
    
    while True:
        try:
            index = int(input("Enter the index of the .pcd file to open: "))
            if 0 <= index < len(files):
                return files[index]
            else:
                print("Invalid index. Please try again.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def main():
    files = list_pcd_files()
    
    if not files:
        print("No .pcd files found in the current directory.")
        return
    
    selected_file = select_pcd_file(files)
    print(f"Opening {selected_file}...")

    # Load the selected point cloud
    pcd = o3d.io.read_point_cloud(selected_file)

    # Initialize Open3D visualizer
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(width=600, height=400)


    vis.add_geometry(pcd)

    def exit_callback(vis):
        vis.destroy_window()

    vis.register_key_callback(81, exit_callback)  # 'q' key

    vis.run()

if __name__ == "__main__":
    main()

