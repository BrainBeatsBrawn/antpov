#!/bin/bash

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
