import open3d as o3d

# Read point cloud from file
point_cloud = o3d.io.read_point_cloud("data/above_table/0929/1.pcd")   

# Visualize the point cloud 
o3d.visualization.draw_geometries([point_cloud])