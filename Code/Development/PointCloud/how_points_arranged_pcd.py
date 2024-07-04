import open3d as o3d
from datetime import datetime
import numpy as np

def sort_point_cloud_by_pca(pcd):
    # Convert Open3D PointCloud to numpy array
    points = np.asarray(pcd.points)

    # Compute centroid
    centroid = np.mean(points, axis=0)

    # Compute covariance matrix
    covariance_matrix = np.cov(points.T)

    # Compute eigenvalues and eigenvectors of covariance matrix
    eigen_values, eigen_vectors = np.linalg.eigh(covariance_matrix)

    # Sort eigenvalues and eigenvectors in descending order
    sort_indices = np.argsort(eigen_values)[::-1]
    eigen_values = eigen_values[sort_indices]
    eigen_vectors = eigen_vectors[:, sort_indices]

    # Use the first three eigenvectors as principal axes
    principal_axes = eigen_vectors[:, :3]

    # Sort points based on the first principal axis
    sorted_indices = np.argsort(np.dot(points - centroid, principal_axes[:, 0]))

    # Create a new point cloud with sorted points
    sorted_points = points[sorted_indices]
    sorted_colors = np.asarray(pcd.colors)[sorted_indices]
    sorted_pcd = o3d.geometry.PointCloud()
    sorted_pcd.points = o3d.utility.Vector3dVector(sorted_points)
    sorted_pcd.colors = o3d.utility.Vector3dVector(sorted_colors)

    return sorted_pcd

def main():
    # Initialize Open3D visualizer
    vis = o3d.visualization.VisualizerWithKeyCallback()
    vis.create_window(width=600, height=400)

    # Load the PCD file
    pcd = o3d.io.read_point_cloud("point_cloud_20240702_190339.pcd")  # Replace with your PCD file path
    
    # Rotate the pointcloud to verify this works for any "oreintation". The only test that is not done now is if we have a very bent rope. or say a looped back on itself type thing. 
    R = pcd.get_rotation_matrix_from_xyz((0, 0, np.pi/2))
    pcd.rotate(R, center=(0,0,0))
    
    # Sort point cloud based on PCA
    sorted_pcd = sort_point_cloud_by_pca(pcd)

    # Initialize an empty point cloud for visualization
    vis_pcd = o3d.geometry.PointCloud()

    # Add a fixed point (for demonstration)
    fixed_point = [0.0, 0.0, 0.0]
    vis_pcd.points.append(fixed_point)
    vis_pcd.colors.append([1.0, 1.0, 1.0])  # White color for fixed point

    # Register 'q' key for exit callback
    vis.register_key_callback(81, lambda vis: vis.destroy_window())

    try:
        # Add the sorted points from PCD file
        vis.add_geometry(sorted_pcd)  # Add the sorted point cloud once

        for i in range(len(sorted_pcd.points)):
            dt0 = datetime.now()

            # Add points iteratively to vis_pcd
            vis_pcd.points.append(sorted_pcd.points[i])
            vis_pcd.colors.append([0.0, 0.0, 1.0])  # Blue color

            # Update visualization
            if i == 0:
                vis.add_geometry(vis_pcd)
            else:
                vis.update_geometry(vis_pcd)
                vis.poll_events()
                vis.update_renderer()

            # Calculate and print FPS
            dt1 = datetime.now()
            delta_t = dt1 - dt0
            fps = 1.0 / delta_t.total_seconds()
            print(f"FPS: {fps:.2f}")

    finally:
        vis.destroy_window()

if __name__ == "__main__":
    main()

