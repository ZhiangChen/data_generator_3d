import bpy
import os

# Set the path where you want to export the meshes
export_path = "output/object_meshes"
if not os.path.exists(export_path):
    os.makedirs(export_path)

# Load the Blender file generated from Kubric (replace with your actual file path)
bpy.ops.wm.open_mainfile(filepath="output/semantic_SfM.blend")

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
