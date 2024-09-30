import numpy as np
import open3d as o3d

# Read point cloud from file
point_cloud = o3d.io.read_point_cloud("data/above_table/d455/5.pcd")
points = np.asarray(point_cloud.points)

# Remove points that are below the table
points = points[points[:, 1] < 0.5]

# Visualize the point cloud
point_cloud.points = o3d.utility.Vector3dVector(points)
o3d.visualization.draw_geometries([point_cloud])