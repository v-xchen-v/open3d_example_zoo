import open3d as o3d
import numpy as np

def compute_centroid(pcd):
    """Computes the centroid of a point cloud."""
    return np.mean(np.asarray(pcd.points), axis=0)

def move_source_to_target(source, target):
    """Translates the source point cloud to the target's centroid."""
    source_centroid = compute_centroid(source)
    target_centroid = compute_centroid(target)
    translation = target_centroid - source_centroid  # Calculate translation vector
    source.translate(translation)  # Move source to target's center
    return source, translation

def icp_align(source, target, threshold=0.02, max_iterations=50):
    """Performs ICP alignment between source and target."""
    # Initialize with identity matrix as the transformation guess
    init_transformation = np.identity(4)

    # Apply ICP
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, init_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iterations)
    )

    # print("ICP converged:", reg_p2p.converged)
    print("ICP Fitness:", reg_p2p.fitness)
    print("ICP Transformation:\n", reg_p2p.transformation)



    return reg_p2p.transformation

def restore_source(source, translation):
    """Restores the source to its original position."""
    source.translate(-translation)  # Reverse the initial translation
    return source

def align_source_to_target(source, target):
    # Step 1: Move the source to the target's centroid
    source_moved, translation = move_source_to_target(source, target)

    # Step 2: Perform ICP alignment
    transformation = icp_align(source_moved, target)

    # Step 3: Apply ICP transformation to the source
    source_moved.transform(transformation)

    # Step 4: Restore the source to its original position
    # aligned_source = restore_source(source_moved, translation)

    return source_moved

# Example usage
if __name__ == "__main__":
    # Load source and target point clouds (replace with your file paths)
    source = o3d.io.read_point_cloud("data/coke_can.pcd") # the full view of the coke can
    target = o3d.io.read_point_cloud("data/pcd/stanford_dataset/bunny.pcd") # the partial view of the coke can

    # Align the source to the target
    aligned_source = align_source_to_target(source, target)
    
    # Compute the centroid of the aligned source
    aligned_centroid = compute_centroid(aligned_source)

    # Visualize the alignment result
    o3d.visualization.draw_geometries([aligned_source, target])
