import bpy
import os
import sys

# Set the path where you want to export the meshes
data_id = 0
if len(sys.argv) > 1:
    data_id = int(sys.argv[1])

export_path = f"output/kubric_{data_id}/object_meshes"
if not os.path.exists(export_path):
    os.makedirs(export_path)

# Load the Blender file generated from Kubric (replace with your actual file path)
blender_file = f"output/kubric_{data_id}/semantic_SfM.blend"
bpy.ops.wm.open_mainfile(filepath=blender_file)

# Set the scene to the last frame
scene = bpy.context.scene
last_frame = scene.frame_end
scene.frame_set(last_frame)

# Loop through each object in the scene and export it as a mesh
for id, obj in enumerate(bpy.data.objects):
    # Only export mesh objects (ignore cameras, lights, etc.)
    if obj.type == 'MESH':
        # Select the object
        bpy.ops.object.select_all(action='DESELECT')  # Deselect all objects
        obj.select_set(True)  # Select the object
        
        # Define export filename using id as ply files
        export_filename = os.path.join(export_path, f"{id}.ply")

        # Export the object as a mesh
        bpy.ops.export_mesh.ply(filepath=export_filename, use_selection=True)
    

print("All objects exported.")
