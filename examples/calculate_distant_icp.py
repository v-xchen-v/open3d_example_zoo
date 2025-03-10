import open3d as o3d
import numpy as np

def center_point_cloud(pcd):
    """Centers the point cloud by subtracting the centroid."""
    centroid = np.mean(np.asarray(pcd.points), axis=0)
    pcd.translate(-centroid)  # Move to the origin
    return pcd, centroid

def icp_align(source, target, threshold=0.02, max_iterations=50):
    """Performs ICP alignment on two centered point clouds."""
    # Estimate initial transformation matrix (identity as starting point)
    init_transformation = np.identity(4)

    # Apply ICP
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, init_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations),
    )

    # print("ICP converged:", reg_p2p.converged)
    print("ICP Fitness:", reg_p2p.fitness)
    print("ICP Transformation:\n", reg_p2p.transformation)

    return reg_p2p.transformation

def restore_point_cloud(pcd, centroid):
    """Restores the point cloud to its original position."""
    pcd.translate(centroid)  # Move back to the original position
    return pcd

def align_and_restore(source, target):
    # Step 1: Center both point clouds at the origin
    source_centered, source_centroid = center_point_cloud(source)
    target_centered, target_centroid = center_point_cloud(target)

    # Step 2: Perform ICP alignment on centered point clouds
    transformation = icp_align(source_centered, target_centered)

    # Step 3: Apply transformation to the original (non-centered) source
    source_centered.transform(transformation)

    # Step 4: Restore both point clouds to their original positions
    aligned_source = restore_point_cloud(source_centered, source_centroid)
    restored_target = restore_point_cloud(target_centered, target_centroid)

    return aligned_source, restored_target

# Example usage
if __name__ == "__main__":
    # Load point clouds (replace with your paths)
    source = o3d.io.read_point_cloud("data/pcd/stanford_dataset/bunny.pcd")
    target = o3d.io.read_point_cloud("data/coke_can.pcd")

    # Align and restore point clouds
    aligned_source, restored_target = align_and_restore(source, target)

    # Visualize the result
    o3d.visualization.draw_geometries([aligned_source, restored_target])
