import open3d as o3d
import numpy as np

# Load or generate two point clouds
pcd1 = o3d.io.read_point_cloud("data/pcd/stanford_dataset/blade.pcd")  # Source point cloud
pcd2 = o3d.io.read_point_cloud("data/pcd/stanford_dataset/blade.pcd")  # Target point cloud

# Preprocess point clouds (e.g., downsample)
voxel_size = 0.005
pcd1_downsampled = pcd1.voxel_down_sample(voxel_size)
pcd2_downsampled = pcd2.voxel_down_sample(voxel_size)

# Compute normals if necessary (for ICP with point-to-plane)
pcd1_downsampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
pcd2_downsampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

# Initial alignment (e.g., with global registration or just using the identity matrix as initial guess)
initial_transform = np.identity(4)  # You can use global alignment here if needed

# Apply the ICP algorithm (point-to-point or point-to-plane)
# Point-to-point ICP
icp_result = o3d.pipelines.registration.registration_icp(
    pcd1_downsampled, pcd2_downsampled, max_correspondence_distance=0.1,
    init=initial_transform,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint())

# ICP result and transformation matrix
print("ICP has converged:", icp_result.transformation)
print("Fitness score:", icp_result.fitness)
print("Final transformation matrix:")
print(icp_result.transformation)

# Transform the source point cloud to align with the target point cloud
pcd1.transform(icp_result.transformation)

# Visualize the aligned point clouds
o3d.visualization.draw_geometries([pcd1, pcd2])
