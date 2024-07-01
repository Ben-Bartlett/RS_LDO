import open3d as o3d
import numpy as np
from scipy.interpolate import splprep, splev
import os

def list_pcd_files():
    files = [os.path.join('./bags', f) for f in os.listdir('./bags') if f.endswith('.pcd')]
    return files

def select_pcd_file(files):
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

def downsample_point_cloud(pcd, voxel_size=0.02):
    downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return downsampled_pcd

def sort_and_average_points(points, bin_size=10):
    points = points[np.argsort(points[:, 0])]
    averaged_points = []
    for i in range(0, len(points), bin_size):
        bin_points = points[i:i+bin_size]
        averaged_point = np.mean(bin_points, axis=0)
        averaged_points.append(averaged_point)
    return np.array(averaged_points)

def fit_spline_to_points(points, num_points=100):
    tck, u = splprep([points[:, 0], points[:, 1], points[:, 2]], s=0)
    u_fine = np.linspace(0, 1, num_points)
    x_fine, y_fine, z_fine = splev(u_fine, tck)
    spline_points = np.vstack((x_fine, y_fine, z_fine)).T
    return spline_points, tck

def add_cylinders_between_points(vis, spline_points, tck):
    cylinders = []
    radius = 0.005  # Adjust as needed

    for i in range(len(spline_points) - 1):
        start_point = spline_points[i]
        end_point = spline_points[i + 1]

        direction = end_point - start_point
        length = np.linalg.norm(direction)
        
        mid_point = (start_point + end_point) / 2
        origin_to_mid_point = mid_point - np.array([0, 0, 0])
        origin_to_mid_point_unit = origin_to_mid_point / np.linalg.norm(origin_to_mid_point)
        offset_mid_point = mid_point + origin_to_mid_point_unit * radius

        cylinder = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length)
        
        z_axis = np.array([0, 0, 1])
        axis = np.cross(z_axis, direction)
        angle = np.arccos(np.dot(z_axis, direction / length))
        
        if np.linalg.norm(axis) > 0:
            axis /= np.linalg.norm(axis)
            R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
            cylinder.rotate(R, center=(0, 0, 0))
        
        cylinder.translate(offset_mid_point)
        vis.add_geometry(cylinder)
        cylinders.append(cylinder)
    
    return cylinders

def toggle_geometry(vis, geometry_list):
    for geom in geometry_list:
        bool_remove = vis.remove_geometry(geom)
        if not bool_remove:
            vis.add_geometry(geom)

    view_ctl = vis.get_view_control()
    view_ctl.set_front([-0.63246362951347912, 0.39482011410282669, 0.66641341136149679])
    view_ctl.set_lookat([-0.0096679043630299253, -0.017195244617371885, -0.28212898813919807])
    view_ctl.set_up([0.16483440358395104, 0.90923979408575517, -0.38224680017760365])
    view_ctl.set_zoom(0.69999999999999996)

def main():
    files = list_pcd_files()
    if not files:
        print("No .pcd files found in the current directory.")
        return
    selected_file = select_pcd_file(files)
    print(f"Opening {selected_file}...")

    pcd = o3d.io.read_point_cloud(selected_file)
    downsampled_pcd = downsample_point_cloud(pcd, voxel_size=0.004)
    points = np.asarray(downsampled_pcd.points)

    if points.shape[0] <= 3:
        print("Not enough points to fit a spline.")
        return

    averaged_points = sort_and_average_points(points, bin_size=20)
    spline_points, tck = fit_spline_to_points(averaged_points)

    lines = [[i, i + 1] for i in range(len(spline_points) - 1)]
    spline_lineset = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(spline_points),
        lines=o3d.utility.Vector2iVector(lines)
    )
    spline_lineset.paint_uniform_color([0, 0, 1])

    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(width=800, height=600)
    vis.add_geometry(downsampled_pcd)
    vis.add_geometry(spline_lineset)

    cylinders = add_cylinders_between_points(vis, spline_points, tck)

    def exit_callback(vis):
        vis.destroy_window()

    def toggle_pointcloud_callback(vis):
        toggle_geometry(vis, [pcd])

    def toggle_cylinders_callback(vis):
        toggle_geometry(vis, cylinders)

    vis.register_key_callback(ord('Q'), exit_callback)
    vis.register_key_callback(ord('M'), toggle_pointcloud_callback)
    vis.register_key_callback(ord('C'), toggle_cylinders_callback)
    vis.run()

if __name__ == "__main__":
    main()

