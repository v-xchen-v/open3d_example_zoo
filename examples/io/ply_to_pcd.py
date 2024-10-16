import open3d as o3d

# Load a mesh file (example with a .ply file)
mesh = o3d.io.read_triangle_mesh("your_mesh_file.ply")

# Check if the mesh has vertex normals or faces
if not mesh.has_vertex_normals():
    mesh.compute_vertex_normals()

# Sample points uniformly from the mesh surface
# You can specify the number of points you want to sample
point_cloud = mesh.sample_points_uniformly(number_of_points=10000)

# Visualize the point cloud
o3d.visualization.draw_geometries([point_cloud])

# Save the point cloud to a file if necessary (e.g., in .ply format)
o3d.io.write_point_cloud("sampled_point_cloud.ply", point_cloud)
