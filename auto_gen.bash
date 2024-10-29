#!/bin/bash


# Loop over the range of parameters (0 to 9)
for i in {0..129}
do
  echo "Running with parameter: $i"
  
  # Run the first script in the Docker container
  docker run --rm --interactive \
             --user $(id -u):$(id -g) \
             --volume "$(pwd):/kubric" \
             kubricdockerhub/kubruntu \
             /usr/bin/python3 data_generator_3d/generate_data.py $i
  
  # Run the second script in the Docker container
  docker run --rm --interactive \
             --user $(id -u):$(id -g) \
             --volume "$(pwd):/kubric" \
             kubricdockerhub/kubruntu \
             /usr/bin/python3 data_generator_3d/blender2mesh.py $i
  
  # Run the third script locally
  python3 data_generator_3d/mesh2pointcloud.py $i
done
