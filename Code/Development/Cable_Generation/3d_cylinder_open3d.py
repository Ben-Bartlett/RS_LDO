import open3d as o3d
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

# Example usage
if __name__ == "__main__":
    radius = 0.5  # 10mm rope diameter / 2
    length = 30.0  # 30cm length of the rope
    resolution = 30  # Number of points along the length

    # Create Open3D visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=800, height=600)

    # Create cylinder mesh, lineset, and points
    lineset, cylinder_mesh, points = create_cylinder_mesh(radius, length, resolution)

    # Create Open3D point cloud for visualizing points
    points_cloud = o3d.geometry.PointCloud()
    points_cloud.points = o3d.utility.Vector3dVector(points)
    points_cloud.colors = o3d.utility.Vector3dVector(np.tile([1, 0, 0], (resolution, 1)))  # Red color for points

    # Add geometries to the visualizer
    vis.add_geometry(cylinder_mesh)
    vis.add_geometry(lineset)
    vis.add_geometry(points_cloud)

    # Set the initial view parameters
    view_ctl = vis.get_view_control()
    view_ctl.set_front([0.63070819460673366, 0.1117903548955366, 0.76792583613799881])
    view_ctl.set_lookat([-2.3682354801706111, 0.60629051724710314, 1.464558813826659])
    view_ctl.set_up([-0.027122423889536812, 0.99214047123466531, -0.12215424454603543])
    view_ctl.set_zoom(0.54)

    # Run the visualizer
    vis.run()
    vis.destroy_window()
