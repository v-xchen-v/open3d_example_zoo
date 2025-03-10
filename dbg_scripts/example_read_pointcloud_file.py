import open3d as o3d

# Read point cloud from file
point_cloud = o3d.io.read_point_cloud("data/coke_can.pcd")   

# Visualize the point cloud 
o3d.visualization.draw_geometries([point_cloud])