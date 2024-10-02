import logging
import os
import numpy as np
import kubric as kb
from kubric.renderer.blender import Blender as KubricBlender
from kubric.simulator.pybullet import PyBullet as KubricSimulator

logging.basicConfig(level="INFO")  # < CRITICAL, ERROR, WARNING, INFO, DEBUG


image_size = 512  # Image size in pixels (width and height)
simulation_time = 8  # Simulation time in seconds
friction = 1.0  # Friction coefficient
restitution = 0.01  # Restitution coefficient

basic_objects = 100 # Number of basic objects
shapenet_objects = 100 # Number of shapenet objects

floor_size = 400  # Size of the floor in meters
aoi_size = 100  # Area of interest size in meters
camera_height = 30  # Camera height in meters
obj_initi_height = 12.0  # Initial object height in meters
camera_tilting = 10 # Camera tilting angle in degrees


def generate_basic_objects(scene, num_objects=10):
    # --- Load asset files
    assets_dir = "kubric_assets/KuBasic"
    asset_file_paths = []
    for root, dirs, files in os.walk(assets_dir):
        for file in files:
            if file.endswith(('.obj')):
                asset_file_paths.append(os.path.join(root, file))

    # --- Create a random number generator
    rng = np.random.default_rng()

    for _ in range(num_objects):
        # Randomly select an asset file path
        asset_file_path = rng.choice(asset_file_paths)

        # Verify the asset file path
        if not os.path.exists(asset_file_path):
            raise FileNotFoundError(f"Asset file not found: {asset_file_path}")

        random_position = rng.uniform(-aoi_size, aoi_size, size=2)
        random_position = np.append(random_position, obj_initi_height)

        # simulation_filename has the file of object.urdf in the same directory of asset_file_path
        simulation_filename = os.path.join(os.path.dirname(asset_file_path), "object.urdf")


        # Create the object using FileBasedObject
        obj = kb.FileBasedObject(
            asset_id=os.path.basename(asset_file_path),
            render_filename=asset_file_path,
            simulation_filename=simulation_filename,  # Set to None to use default collision shape
            position=random_position,
            quaternion=kb.random_rotation(rng=rng),
            scale=rng.uniform(2, 10),
            mass = 1.0,
        )

        # Set the object's friction and restitution
        obj.friction = friction
        obj.restitution = restitution

        obj.cast_shadows = False

        obj.static = False

        # Add random color to the object
        random_color = kb.Color(
            r=rng.uniform(0.0, 1.0),
            g=rng.uniform(0.0, 1.0),
            b=rng.uniform(0.0, 1.0)
        )
        obj.material = kb.PrincipledBSDFMaterial(color=random_color)

        # Add object to the scene
        scene += obj

def generate_shapenet_objects(scene, num_objects=10):
    # logging generating shapenet objects
    logging.info(f"Generating {num_objects} shapenet objects")
    source_path = "gs://kubric-unlisted/assets/ShapeNetCore.v2.json" 
    shapenet = kb.AssetSource.from_manifest(source_path)

    selected_objects_file = "data_generator_3d/selected_shapenet_objects.txt"
    with open(selected_objects_file, "r") as f:
        selected_objects = f.readlines()

    # remove the newline character from the selected_objects list
    selected_objects = [obj.strip() for obj in selected_objects]

    # randomly add objects from shapenet._assets.items() to the scene
    rng = np.random.default_rng()

    i = num_objects
    while i > 0:
        # Randomly select an asset from the selected_objects list
        asset_id = rng.choice(selected_objects)
        obj = shapenet.create(asset_id=asset_id)

        logging.info(f"selected '{asset_id}'")

        random_position = rng.uniform(-aoi_size, aoi_size, size=2)
        random_position = np.append(random_position, obj_initi_height)
        # make object flat on X/Y and not penetrate floor
        obj.quaternion = kb.Quaternion(axis=[1, 0, 0], degrees=90)
        obj.position = random_position

        # set physics properties
        obj.friction = friction
        obj.restitution = restitution
        obj.mass = 1.0

        obj.cast_shadows = False

        # set random scale 
        obj.scale = rng.uniform(8., 10.0)

        scene.add(obj)
        i -= 1


def render_scene_from_multiple_views(scene, renderer, camera_poses):
    """Render the scene from different camera perspectives."""
    cameras = dict()
    invalid_depth_cameras = []
    for idx, cam_pos in enumerate(camera_poses):
        cam_position = cam_pos[:3]
        cam_tilting = cam_pos[3]
        # Set camera position for this view
        scene.camera.position = cam_position
        # Set camera orientation
        scene.camera.quaternion = kb.Quaternion(axis=[1, 0, 0], degrees=cam_tilting)
        
        # Render the scene for this camera position
        frames_dict = renderer.render()

        # Create output directories if it doesn't exist
        os.makedirs("output/photos", exist_ok=True)
        os.makedirs("output/reconstructions", exist_ok=True)
        os.makedirs("output/associations/depth", exist_ok=True)
        os.makedirs("output/segmentations_gt", exist_ok=True)
        
        # Save the outputs for this camera position
        output_filenames = {
            'rgba': f"output/photos/{idx}.png",
            'depth': f"output/associations/depth/{idx}.png",
            'segmentation': f"output/segmentations_gt/{idx}.png",
        }

        kb.write_png(frames_dict['rgba'][0], output_filenames['rgba'])
        kb.write_palette_png(frames_dict['segmentation'][0], output_filenames['segmentation'])
        depth_scale = kb.write_scaled_png(frames_dict['depth'][0], output_filenames['depth'])
        # save the depth without scaling as npy file
        np.save(f"output/associations/depth/{idx}.npy", frames_dict['depth'][0])
        np.save(f"output/segmentations_gt/{idx}.npy", frames_dict['segmentation'][0])
        logging.info(f"View {idx}: Saved RGBA, depth, and segmentation with depth scale: {depth_scale}")

        if depth_scale['max'] > 1000:
            invalid_depth_cameras.append(idx)

        
        # Save camera intrinsics and extrinsics
        camera_intrinsics = scene.camera.intrinsics.copy()
        camera_position = scene.camera.position.copy()
        camera_rotation_matrix = scene.camera.rotation_matrix.copy()
        # get camera transformation matrix from camera position and rotation matrix
        camera_transformation_matrix = np.eye(4)
        camera_transformation_matrix[:3, :3] = camera_rotation_matrix
        camera_transformation_matrix[:3, 3] = camera_position
        camera_width = image_size
        camera_height = image_size

        camera = dict()
        camera['intrinsics'] = camera_intrinsics
        camera['extrinsics'] = camera_transformation_matrix
        camera['width'] = camera_width
        camera['height'] = camera_height

        image_key = f"{idx}.png"
        cameras[image_key] = camera

    # Save the camera parameters as npy
    np.save("output/reconstructions/camera_poses.npy", cameras)
    print(invalid_depth_cameras)

def generate_camera_poses(fov, tilting, front_overlap, side_overlap):
    """
    Generate camera poses for a given field of view (fov) and tilting angle.

    Args:
        fov (float): Field of view in radians.
        tilting (float): Tilting angle in degrees.
        front_overlap (float): Front overlap ratio.
        side_overlap (float): Side overlap ratio.
    """
    short_distance = camera_height / np.cos(np.radians(tilting))
    swath = short_distance * np.tan(fov / 2) * 2
    forward_coverage = camera_height * (np.tan(np.radians(tilting)+fov) - np.tan(np.radians(tilting)))
    
    side_step_length = swath * (1 - side_overlap)
    front_step_length = forward_coverage * (1 - front_overlap)

    # Generate camera positions in a grid pattern with side and front overlap within the area of interest (aoi)
    side_steps = int(np.ceil(2 * aoi_size / side_step_length)) + 2
    front_steps = int(np.ceil(2 * aoi_size / front_step_length)) + 2

    print(f"side_steps: {side_steps}, front_steps: {front_steps}")
    print(f"side_step_length: {side_step_length}, front_step_length: {front_step_length}")
    print(f"swath: {swath}, forward_coverage: {forward_coverage}")

    camera_poses = []
    for i in range(side_steps):
        for j in range(front_steps):
            x = -aoi_size - side_step_length + i * side_step_length
            y = -aoi_size - front_step_length + j * front_step_length
            camera_poses.append((x, y, camera_height, tilting))
        tilting = -tilting
    
    return camera_poses

if __name__ == "__main__":

    # --- Create scene and attach a renderer and simulator
    scene = kb.Scene(resolution=(image_size, image_size))
    scene.frame_end = 24*simulation_time   # Number of frames to simulate
    scene.frame_rate = 24  # Rendering framerate; setting rendering to 24fps.
    scene.step_rate = 240  # Simulation framerate; setting simulation to 240fps. 
    renderer = KubricBlender(scene)
    simulator = KubricSimulator(scene)

    # --- Populate the scene with objects, lights, and camera
    # Floor with low restitution to minimize bouncing
    floor = kb.Cube(
        name="floor", scale=(floor_size, floor_size, 0.1), position=(0, 0, -0.1), static=True
    )
    floor.friction = friction
    floor.restitution = restitution  # Low restitution

    floor_color = kb.Color(0.2, 0.2, 0.2)  # Define the color (e.g., green)
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


    # Camera setup
    scene.camera = kb.PerspectiveCamera(
        name="camera", 
        position=(3, -1, camera_height), 
        look_at=(0, 0, 1),
        focal_length=30,
    )


    # Generate basic objects
    generate_basic_objects(scene, num_objects=basic_objects)

    # Generate shapenet objects
    generate_shapenet_objects(scene, num_objects=shapenet_objects)

    # --- Run the simulation
    simulator.run()

    # --- Set the scene to render only the last frame
    scene.frame_start = scene.frame_end  # Render only the last frame

    os.makedirs("output", exist_ok=True)
    renderer.save_state("output/semantic_SfM.blend")

    fov = scene.camera.field_of_view

    camera_poses = generate_camera_poses(fov, tilting=camera_tilting, front_overlap=0.85, side_overlap=0.65)

    # print out the generated camera poses number
    logging.info(f"Generated {len(camera_poses)} camera poses")

    render_scene_from_multiple_views(scene, renderer, camera_poses)
