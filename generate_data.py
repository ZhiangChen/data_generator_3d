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

        # set random scale 
        obj.scale = rng.uniform(1., 5.0)

        scene.add(obj)

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
        intensity=1.5,             # Adjust intensity as needed
        shadow_softness=0.5,       # Soften shadows
    )
    scene += sun_light

    # Camera setup
    scene.camera = kb.PerspectiveCamera(
        name="camera", position=(3, -1, camera_height), look_at=(0, 0, 1)
    )

    # Generate basic objects
    generate_basic_objects(scene, num_objects=10)

    # Generate shapenet objects
    generate_shapenet_objects(scene, num_objects=10)

    # --- Run the simulation
    simulator.run()

    # --- Set the scene to render only the last frame
    scene.frame_start = scene.frame_end  # Render only the last frame

    # --- Render the last frame
    os.makedirs("output", exist_ok=True)
    renderer.save_state("output/semantic_SfM.blend")
    frames_dict = renderer.render()

    # Since only one frame is rendered, it's at index 0
    last_frame_data = {}
    for output_type, data_array in frames_dict.items():
        last_frame_data[output_type] = data_array[0]  # First (and only) frame

    # --- Save the outputs
    output_filenames = {
        'rgba': 'output/last_frame_rgba.png',
        'depth': 'output/last_frame_depth.png',
        'segmentation': 'output/last_frame_segmentation.png',
    }

    kb.write_png(last_frame_data['rgba'], output_filenames['rgba'])
    kb.write_palette_png(
        last_frame_data['segmentation'], output_filenames['segmentation']
    )
    depth_scale = kb.write_scaled_png(
        last_frame_data['depth'], output_filenames['depth']
    )
    logging.info(f"Last frame depth scale: {depth_scale}")

    # save depth image without scale as numpy array
    np.save('output/last_frame_depth.npy', last_frame_data['depth'])


