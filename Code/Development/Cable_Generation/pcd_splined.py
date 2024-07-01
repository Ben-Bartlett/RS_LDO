import open3d as o3d
import numpy as np
from scipy.interpolate import splprep, splev
import os

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


def downsample_point_cloud(pcd, voxel_size=0.01):
    downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return downsampled_pcd

def sort_and_average_points(points, bin_size=10):
    # Sort points by their x-coordinates
    points = points[np.argsort(points[:, 0])]

    # Bin points and compute the average of each bin
    averaged_points = []
    for i in range(0, len(points), bin_size):
        bin_points = points[i:i+bin_size]
        averaged_point = np.mean(bin_points, axis=0)
        averaged_points.append(averaged_point)
    
    return np.array(averaged_points)

def fit_spline_to_points(points, num_points=100):
    # Fit the spline using the averaged points
    tck, u = splprep([points[:, 0], points[:, 1], points[:, 2]], s=0)
    u_fine = np.linspace(0, 1, num_points)
    x_fine, y_fine, z_fine = splev(u_fine, tck)
    return np.vstack((x_fine, y_fine, z_fine)).T

def toggle_point_cloud(vis, pcd):
    
    bool_remove = vis.remove_geometry(pcd)

    if bool_remove == False:
        vis.add_geometry(pcd)   
                   


def main():
    files = list_pcd_files()
    if not files:
        print("No .pcd files found in the current directory.")
        return
    selected_file = select_pcd_file(files)
    print(f"Opening {selected_file}...")

    # Load the selected point cloud
    pcd = o3d.io.read_point_cloud(selected_file)

    # Downsample the point cloud to reduce the number of points
    downsampled_pcd = downsample_point_cloud(pcd, voxel_size=0.005)
    points = np.asarray(downsampled_pcd.points)

    if points.shape[0] <= 3:
        print("Not enough points to fit a spline.")
        return

    # Sort and average points
    averaged_points = sort_and_average_points(points, bin_size=20)

    # Fit a spline to the averaged points
    spline_points = fit_spline_to_points(averaged_points)

    # Create a line set for the spline
    lines = [[i, i + 1] for i in range(len(spline_points) - 1)]
    spline_lineset = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(spline_points),
        lines=o3d.utility.Vector2iVector(lines)
    )
    spline_lineset.paint_uniform_color([0, 0, 1])  # Blue color for the spline

    # Initialize Open3D visualizer
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(width=800, height=600)

    # Add the original point cloud and the fitted spline to the visualizer
    vis.add_geometry(pcd)
    vis.add_geometry(spline_lineset)

    def exit_callback(vis):
        vis.destroy_window()

    # Register 'm' key to toggle visibility of the point cloud
    def toggle_callback(vis):
        toggle_point_cloud(vis, pcd)


    vis.register_key_callback(81, exit_callback)  # 'q' key
    vis.register_key_callback(77, toggle_callback)  # 'm' key
    vis.run()

if __name__ == "__main__":
    main()


