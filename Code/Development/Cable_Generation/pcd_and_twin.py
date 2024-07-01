import open3d as o3d
import os
import numpy as np

# Function to create cylinder mesh, line set, and points
def create_cylinder_mesh(radius, length, resolution=30):
    # Create points along one edge of the cylinder from one end face to another
    z = np.linspace(0, length, resolution)

    # Points along the cylinder edge on one face, starting from one end
    points = []
    for i in range(resolution):
        points.append([radius, 0, z[i]])  # Points on the positive x face

    # Offset the points so they start from the beginning of the cylinder
    offset = length / 2.0  # Offset by half the length
    points = np.array(points) - np.array([0, 0, offset])

    # Create Open3D lineset for the cylinder's path along one edge
    lines = []
    for i in range(resolution - 1):
        lines.append([i, i + 1])

    line_set = o3d.geometry.LineSet(
        points=o3d.utility.Vector3dVector(points),
        lines=o3d.utility.Vector2iVector(lines)
    )

    # Create Open3D cylinder mesh
    cylinder_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length)
    cylinder_mesh.compute_vertex_normals()

    return line_set, cylinder_mesh, points

def list_pcd_files():
    # List all .pcd files in the current directory
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
    radius = 0.005  # 10mm rope diameter / 2
    length = 0.20  # 20cm length of the rope
    resolution = 30  # Number of points along the length
    
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

    # Add the point cloud
    vis.add_geometry(pcd)


    # Create cylinder mesh, lineset, and points
    lineset, cylinder_mesh, points = create_cylinder_mesh(radius, length, resolution)

    # Rotate the cylinder by 90 degrees around the y-axis
    R = cylinder_mesh.get_rotation_matrix_from_xyz((0, -1*np.pi / 2, 0))
    cylinder_mesh.rotate(R, center=(0, 0, 0))
    lineset.rotate(R, center=(0, 0, 0))
    points = points @ R.T

    # Translate the cylinder 31 cm along the z-axis
    translation = np.array([0.0125, -0.025, -0.32])
    cylinder_mesh.translate(translation)
    lineset.translate(translation)
    points += translation

    # Create Open3D point cloud for visualizing points
    points_cloud = o3d.geometry.PointCloud()
    points_cloud.points = o3d.utility.Vector3dVector(points)
    points_cloud.colors = o3d.utility.Vector3dVector(np.tile([1, 0, 0], (resolution, 1)))  # Red color for points

    # Add geometries to the visualizer
    vis.add_geometry(cylinder_mesh)
    vis.add_geometry(lineset)
    vis.add_geometry(points_cloud)

    # Set the initial view parameters
    def exit_callback(vis):
        vis.destroy_window()

    vis.register_key_callback(81, exit_callback)  # 'q' key

    vis.run()

if __name__ == "__main__":
    main()

