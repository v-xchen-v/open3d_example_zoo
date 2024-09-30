import numpy as np
import open3d as o3d
import cv2
from scipy.spatial.transform import Rotation as R

# Read point cloud from file
point_cloud = o3d.io.read_point_cloud("data/above_table/d455/6.pcd")
points = np.asarray(point_cloud.points)


# load .npy file
rvec_table2cam = np.load("data/calib_table/d455_06/rvec_table2cam.npy")
tvec_table2cam = np.load("data/calib_table/d455_06/tvec_table2cam.npy")

# convert points from camera coordinate to table coordinate
T_table2rgbcam = np.eye(4)
T_table2rgbcam[:3, :3] = R.from_rotvec(rvec_table2cam.flatten()).as_matrix()

print(R.from_rotvec(rvec_table2cam[:, 0]).as_quat())
print(tvec_table2cam.flatten())
T_table2rgbcam[:3, 3] = tvec_table2cam.flatten()
# T_table2rgbcam[:3, 3] = [tvec_table2cam[:, 0][0], tvec_table2cam[:, 0][1], tvec_table2cam[:, 0][2]]
# T_table2rgbcam[:3, 3] = [0, 0, tvec_table2cam[:, 0][2]]
T_rbgcam2table = np.linalg.inv(T_table2rgbcam)

# # a 4x4 matrix of reverse y, z axis
# T_reverse_xz = np.eye(4)
# T_reverse_xz[0, 0] = 1
# T_reverse_xz[2, 2] = 1
# T_pccam2rgbcam = T_reverse_xz

# a 4x4 matrix of rotate 180 degree around x axis
T_rot180_x = np.eye(4)
T_rot180_x[1, 1] = -1
T_rot180_x[2, 2] = -1
# T_pccam2rgbcam = T_rot180_x
# T_rgbcam2pccam = np.linalg.inv(T_pccam2rgbcam)

T_pccam2rgbcam = np.load("data/above_table/d455/T_depth_to_rgb.npy")
T_rbgcam2pccam = np.linalg.inv(T_table2rgbcam)

p_cam = points
# p_tables = np.dot(T_table2cam, np.vstack([p_cam.T, np.ones([1, p_cam.shape[0]])]))[:3]
p_tables = []
for one_point in p_cam:
    one_point = np.append(one_point, 1)
    # convert points from camera coordinate to table coordinate
    # p_table = np.dot(T_table2cam, one_point)[:3]
    # p_table = (T_rbgcam2table@T_pccam2rgbcam@one_point)[:3]
    
    # axis should same as rgb camera
    # p_table = (T_rot180_x@T_pccam2rgbcam@one_point)[:3]
    
    # p_table = T_pccam2rgbcam@pne
    
    # p_table = (T_rbgcam2table@T_pccam2rgbcam@one_point)[:3]
    # p_table = (T_table2rgbcam@T_rbgcam2pccam@one_point)[:3]
    
    
    # p_table = np.dot(T_cam2table, one_point)[:3]
    
    
    # p_table = np.dot(T_pccam2rgbcam, one_point)[:3]
    p_table = (T_rbgcam2table@T_pccam2rgbcam@T_rot180_x@one_point)[:3]
    
    
    p_tables.append(p_table)
p_tables = np.array(p_tables)
# p_tables = points
    
# Remove points that are below the table
p_tables = p_tables[p_tables[:, 2] <-0.02]

# Remove point out of table area
p_tables = p_tables[p_tables[:, 0] > -0.1]
p_tables = p_tables[p_tables[:, 0] < 0.37]
p_tables = p_tables[p_tables[:, 1] > -0.1]
p_tables = p_tables[p_tables[:, 1] < 0.5]

# convert points from table coordinate to camera coordinate
# T_cam2table = np.linalg.inv(T_table2cam)
# p_cam = np.dot(T_cam2table, np.vstack([p_tables.T, np.ones([1, p_tables.shape[0]])]))[:3]


# Visualize the point cloud
point_cloud.points = o3d.utility.Vector3dVector(p_tables)
# o3d.visualization.draw_geometries([point_cloud])

# set visualize view as center of point cloud
vis = o3d.visualization.Visualizer()
vis.create_window()
# add coordinate frame

# # visualize the point cloud with texture color


coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
vis.add_geometry(coord_frame)
vis.add_geometry(point_cloud)
ctr = vis.get_view_control()
ctr.set_lookat([0, 0, 0])
ctr.set_front([0, 0, 1])
ctr.set_up([0, -1, 0])
ctr.set_zoom(0.8)
vis.run()

# draw the coordinate
# coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1, origin=[0, 0, 0])
# o3d.visualization.draw_geometries([point_cloud, coord_frame])

# Save the point cloud
o3d.io.write_point_cloud("data/above_table/d455/6_table.pcd", point_cloud)
# save as ply file
# o3d.io.write_point_cloud("data/above_table/d455/5_table.ply", point_cloud)