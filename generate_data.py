import logging
import os
import numpy as np
import kubric as kb
from kubric.renderer.blender import Blender as KubricBlender
from kubric.simulator.pybullet import PyBullet as KubricSimulator

logging.basicConfig(level="INFO")  # < CRITICAL, ERROR, WARNING, INFO, DEBUG

image_size = 512  # Image size in pixels (width and height)
simulation_time = 5  # Simulation time in seconds
floor_size = 15  # Size of the floor in meters
aoi_size = 10  # Area of interest size in meters
friction = 1.0  # Friction coefficient
restitution = 0.01  # Restitution coefficient
camera_height = 15  # Camera height in meters
obj_initi_height = 2.0  # Initial object height in meters
basic_objects = 5
shapenet_objects = 5 


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
            scale=rng.uniform(0.5, 1.5),
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
    source_path = "gs://kubric-unlisted/assets/ShapeNetCore.v2.json" 
    shapenet = kb.AssetSource.from_manifest(source_path)
    # randomly add objects from shapenet._assets.items() to the scene
    rng = np.random.default_rng()
    for _ in range(num_objects):
        # Randomly select an asset from shapenet
        asset_id = rng.choice(list(shapenet._assets.keys()))
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

        obj.cast_shadows = False

        # set random scale 
        obj.scale = rng.uniform(1., 5.0)

        scene.add(obj)


def render_scene_from_multiple_views(scene, renderer, camera_positions):
    """Render the scene from different camera perspectives."""
    cameras = dict()
    for idx, cam_pos in enumerate(camera_positions):
        # Set camera position for this view
        scene.camera.position = cam_pos
        # Set camera orientation
        scene.camera.quaternion = kb.Quaternion(axis=[0, 0, 1], degrees=0)  # Camera orientation
        
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
        logging.info(f"View {idx}: Saved RGBA, depth, and segmentation with depth scale: {depth_scale}")
        
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

def generate_camera_poses():
    pass

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
        focal_length=35,
    )


    # Generate basic objects
    generate_basic_objects(scene, num_objects=basic_objects)

    # Generate shapenet objects
    generate_shapenet_objects(scene, num_objects=shapenet_objects)

    # --- Run the simulation
    simulator.run()

    # --- Set the scene to render only the last frame
    scene.frame_start = scene.frame_end  # Render only the last frame

    render_scene_from_multiple_views(scene, renderer, camera_positions=[(1, 1, camera_height), (0, 0, camera_height)])

