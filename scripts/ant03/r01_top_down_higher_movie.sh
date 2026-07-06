#!/bin/bash

# Test for data/seville/ground_and_veg_inner_circular.gltf
if [ ! -f data/seville/ground_and_veg_inner_circular.gltf ]; then
    echo "Please download the Seville environment gltf file and its navmesh"
    exit 1
fi
# Test for the navmesh file and warn (the program will re-create it, but at a significant time cost)
if [ ! -f data/seville/navmesh_4420489099394405100 ]; then
    echo "Warning: The Seville environment's navmesh file is not present, this will take about 3 *hours* to regenerate."
    echo "You may wish to stop and download the file!"
fi

# Ant03R01 for zoomed-out top down video with no follow
#
# -d 1450x1800 gives the right aspect ratio
# -x runs as fast as possible
# -m outputs the movie frames
# -F switches off agent-camera-following

./build/antpov -f data/seville/ground_and_veg_inner_circular.gltf \
               -c data/seville/Ant03R01-warped.csv.world.csv \
               -j data/seville/Ant03R01-warped.csv.world.topdown_higher.json \
               -d 1450x1800 -x -m -F
