import logging
import os
import numpy as np
import kubric as kb
from kubric.renderer.blender import Blender as KubricBlender
from kubric.simulator.pybullet import PyBullet as KubricSimulator

logging.basicConfig(level="INFO")

image_size = 512
simulation_time = 5
floor_size = 150
aoi_size = 10
friction = 1.0
restitution = 0.01
camera_height = 10
obj_initi_height = 2.0
max_depth_threshold = 15.0  # Set your depth threshold
num_shapenet_objects_to_find = 4  # Find 5 ShapeNet objects with limited depth rendering

def generate_centered_shapenet_object(scene, shapenet, rng):
    # Randomly select a ShapeNet object
    asset_id = rng.choice(list(shapenet._assets.keys()))
    obj = shapenet.create(asset_id=asset_id)
    logging.info(f"Selected '{asset_id}' for depth evaluation")

    # Position the object at the center of the floor
    obj.position = (0, 0, obj_initi_height)

    # Apply default rotation and physics properties
    obj.quaternion = kb.Quaternion(axis=[1, 0, 0], degrees=90)
    obj.friction = friction
    obj.restitution = restitution

    # Set a random scale
    obj.scale = rng.uniform(1.0, 5.0)

    # Add object to the scene
    scene += obj

    return asset_id, obj

def render_depth_image(scene, renderer, asset_id):
    """Render from top-down and three side views, save depth images, and return the largest depth."""
    # Set the scene to render only the last frame
    scene.frame_start = scene.frame_end
    
    # Camera positions (top-down and four side views)
    camera_views = {
        "top_down": (0, 0, camera_height),  # Directly above the object
        "side_1": (camera_height/3, 0, camera_height),  # Side view 1
        "side_2": (0, camera_height/3, camera_height),  # Side view 2
        "side_3": (-camera_height/3, 0, camera_height),  # Side view 3
        "side_4": (0, -camera_height/3, camera_height),  # Side view 4
    }
    
    max_depth = -np.inf  # Initialize the max depth to a very low value
    
    for view_name, cam_pos in camera_views.items():
        # Set the camera position for each view
        scene.camera.position = cam_pos
        scene.camera.look_at = (0, 0, obj_initi_height)  # Make the camera point to the object center

        # Render the scene (only the last frame)
        frames_dict = renderer.render()

        # Extract the depth frame
        depth_image = frames_dict['depth'][0]

        # Calculate the maximum depth value for this view
        current_max_depth = np.max(depth_image)

        # Update the max depth if the current view's max depth is larger
        max_depth = max(max_depth, current_max_depth)
        
        # Create output directories if they don't exist
        os.makedirs("output/shapenet_depth", exist_ok=True)
        
        # Save the depth image as PNG for this view
        depth_image_path_png = f"output/shapenet_depth/{asset_id}_{view_name}_depth_image.png"
        kb.write_scaled_png(depth_image, depth_image_path_png)

        # write rgba using kb.write_png(frames_dict['rgba'][0], output_filenames['rgba'])
        kb.write_png(frames_dict['rgba'][0], f"output/shapenet_depth/{asset_id}_{view_name}_rgba_image.png")

        
        logging.info(f"Saved depth image for view '{view_name}' to '{depth_image_path_png}'")

    return max_depth



def find_shapenet_objects_with_limited_depth(scene, renderer, shapenet, num_objects_to_find, max_depth_threshold):
    found_objects = []
    rng = np.random.default_rng()
    
    while len(found_objects) < num_objects_to_find:

        # Generate a new ShapeNet object at the center
        asset_id, obj = generate_centered_shapenet_object(scene, shapenet, rng)

        # Render depth image and calculate max depth
        max_depth = render_depth_image(scene, renderer, asset_id)
        logging.info(f"Rendered depth image with max depth: {max_depth}")

        # If the max depth is smaller than the threshold, append the asset ID
        if max_depth < max_depth_threshold:
            found_objects.append(asset_id)
            logging.info(f"Asset '{asset_id}' meets the depth criteria and is added to the list.")

            # append the found_shapenet_objects to a file
            output_file = "data_generator_3d/selected_shapenet_objects.txt"
            with open(output_file, "a") as f:
                f.write(f"{asset_id}\n")

        else:
            logging.info(f"Asset '{asset_id}' does not meet the depth criteria.")
        
        # logging a line for better readability
        logging.info("*" * 50)
        # Remove the object from the scene to reset for the next iteration
        scene.remove(obj)
    
    return found_objects

if __name__ == "__main__":
    select_data = True
    if select_data:
        # --- Create scene and attach a renderer and simulator
        scene = kb.Scene(resolution=(image_size, image_size))
        scene.frame_end = 24 * simulation_time   # Number of frames to simulate
        scene.frame_rate = 24  # Rendering framerate; setting rendering to 24fps.
        scene.step_rate = 240  # Simulation framerate; setting simulation to 240fps. 
        renderer = KubricBlender(scene)
        simulator = KubricSimulator(scene)

        # --- Populate the scene with a floor and camera
        # Floor with low restitution to minimize bouncing
        floor = kb.Cube(
            name="floor", scale=(floor_size, floor_size, 0.1), position=(0, 0, -0.1), static=True
        )
        floor.friction = friction
        floor.restitution = restitution  # Low restitution
        floor_color = kb.Color(0.2, 0.2, 0.2)  # Define the color
        floor.material = kb.PrincipledBSDFMaterial(color=floor_color)
        scene += floor

        # --- Add the sun light
        sun_light = kb.DirectionalLight(
            name="sun_light",
            position=(0, 0, 10),       # High elevation along the Z-axis
            look_at=(0, 0, 0),         # Pointing towards the center of the scene
            intensity=2.0,             # Adjust intensity as needed
            shadow_softness=2.0,       # Soften shadows
        )
        scene += sun_light

        # --- Camera setup above the center, looking downward
        scene.camera = kb.PerspectiveCamera(
            name="camera",
            position=(0, 0, camera_height),  # Directly above the center of the floor
            look_at=(0, 0, 0),               # Pointing down at the center
            focal_length=30,
        )

        # --- Load ShapeNet assets
        source_path = "gs://kubric-unlisted/assets/ShapeNetCore.v2.json"
        shapenet = kb.AssetSource.from_manifest(source_path)

        # --- Find ShapeNet objects with limited depth
        found_shapenet_objects = find_shapenet_objects_with_limited_depth(
            scene, renderer, shapenet, num_shapenet_objects_to_find, max_depth_threshold
        )

        logging.info(f"Found ShapeNet objects with limited depth: {found_shapenet_objects}")

    else:
        # Load the selected ShapeNet objects from the file
        selected_objects_file = "data_generator_3d/selected_shapenet_objects.txt"
        with open(selected_objects_file, "r") as f:
            selected_objects = f.readlines()
            
        selected_objects = [obj.strip() for obj in selected_objects]
        # check if repeated objects are present in the list
        if len(selected_objects) != len(set(selected_objects)):
            # remove duplicates
            selected_objects = list(set(selected_objects))
            # print the length of the list after removing duplicates
            logging.info(f"Removed duplicates from the list. New length: {len(selected_objects)}")
            # write the new list to the file
            with open(selected_objects_file, "w") as f:
                for obj in selected_objects:
                    f.write(f"{obj}\n")
