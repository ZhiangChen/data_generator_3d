import os
import open3d as o3d
import laspy
import numpy as np

def ply_to_las(ply_directory, output_las_file):
    all_points = []
    all_ids = []

    ply_files = [f for f in os.listdir(ply_directory) if f.endswith('.ply')]
    

    # Loop through all the .ply files in the directory
    for id, file_name in enumerate(ply_files):
        # Load the PLY file
        ply_path = os.path.join(ply_directory, file_name)
        # Load the mesh from the file
        mesh = o3d.io.read_triangle_mesh(ply_path)
        # get the bounding box of the mesh
        bbox = mesh.get_axis_aligned_bounding_box()
        # calculate the volume of the bounding box
        volume = bbox.volume()
        # sampling points is linear to the volume of the bounding box
        n_points = int(volume * 1000)

        if n_points < 1000:
            n_points = 1000
        elif n_points > 50000:
            n_points = 50000
        
        print(f"Sampling {n_points} points from {file_name}")
        pcd = mesh.sample_points_poisson_disk(n_points)
        points = np.asarray(pcd.points)
        
        # Append points to the global list
        all_points.append(points)

        # Append repeated ids
        ids = np.full((points.shape[0],), id)  # Correctly shape the ID array
        all_ids.append(ids)



    # Combine all points into a single array
    all_points = np.vstack(all_points)
    print(f"Total number of points in combined cloud: {all_points.shape[0]}")

    # Create a .las file and write the combined points to it
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.offsets = np.min(all_points, axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])  # Example scale

    las = laspy.LasData(header)
    las.x = all_points[:, 0]
    las.y = all_points[:, 1]
    las.z = all_points[:, 2]

    # Add ids as intensity
    all_ids = np.hstack(all_ids)
    las.intensity = all_ids

    # Write the points to the output .las file
    las.write(output_las_file)
    print(f"Saved combined point cloud to {output_las_file}")

# Usage
ply_directory = "output/object_meshes"  # Path where your .ply files are stored
output_las_file = "output/combined_point_cloud.las"  # Output .las file
ply_to_las(ply_directory, output_las_file)
