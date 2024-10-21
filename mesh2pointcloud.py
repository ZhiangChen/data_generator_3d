import os
import open3d as o3d
import laspy
import numpy as np
from joblib import Parallel, delayed

def process_ply_file(file_name, ply_directory, id):
    # Load the PLY file
    ply_path = os.path.join(ply_directory, file_name)
    # Load the mesh from the file
    mesh = o3d.io.read_triangle_mesh(ply_path)
    # Get the bounding box of the mesh
    bbox = mesh.get_axis_aligned_bounding_box()
    # Calculate the volume of the bounding box
    volume = bbox.volume()
    # Sampling points based on the volume of the bounding box
    n_points = int(volume * 20)

    if n_points < 500:
        n_points = 500
    elif n_points > 600000:
        n_points = 1500000

    print(f"Sampling {n_points} points from {file_name}")
    pcd = mesh.sample_points_poisson_disk(n_points)
    points = np.asarray(pcd.points)

    # Create IDs for the points
    ids = np.full((points.shape[0],), id)  # Correctly shape the ID array

    return points, ids

def ply_to_las_parallel(ply_directory, output_las_file, n_jobs=-1):
    ply_files = [f for f in os.listdir(ply_directory) if f.endswith('.ply')]

    # Use joblib to parallelize the process
    results = Parallel(n_jobs=n_jobs)(
        delayed(process_ply_file)(file_name, ply_directory, id) for id, file_name in enumerate(ply_files)
    )

    # Separate the results into points and ids
    all_points, all_ids = zip(*results)

    # Combine all points into a single array
    all_points = np.vstack(all_points)
    all_ids = np.hstack(all_ids)

    print(f"Total number of points in combined cloud: {all_points.shape[0]}")

    # Create a .las file and write the combined points to it
    header = laspy.LasHeader(point_format=3, version="1.2")
    #header.offsets = np.min(all_points, axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])  # Example scale

    las = laspy.LasData(header)
    las.x = all_points[:, 0]
    las.y = all_points[:, 1]
    las.z = all_points[:, 2]

    # Add ids as intensity
    las.intensity = all_ids

    # Write the points to the output .las file
    las.write(output_las_file)
    print(f"Saved combined point cloud to {output_las_file}")


if __name__ == "__main__":
    # read parameter from the command line
    import sys
    if len(sys.argv) > 1:
        data_id = int(sys.argv[1])
    else:
        data_id = 0

    print(f"Processing data with ID {data_id}")
    ply_directory = f"output/kubric_{data_id}/object_meshes"  # Directory containing .ply files
    output_las_file = f"output/kubric_{data_id}/reconstructions/combined_point_cloud.las"  # Output .las file
    ply_to_las_parallel(ply_directory, output_las_file, n_jobs=6)
