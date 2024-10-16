import numpy as np
import open3d as o3d

# Load the .npy file
point_cloud_array = np.load('data/coke_can.npy')

# Create an Open3D point cloud object
point_cloud_o3d = o3d.geometry.PointCloud()

# Assign points from numpy array to the Open3D point cloud
point_cloud_o3d.points = o3d.utility.Vector3dVector(point_cloud_array)

# Save the point cloud as a .pcd file
o3d.io.write_point_cloud('data/coke_can.pcd', point_cloud_o3d)
