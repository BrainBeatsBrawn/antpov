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

# Ant 06, runs 4, 5, 6, 7 and the zero vector path. (Runs 1 and 2 were tiny, quickly returning to the nest, run 3 is absent in the data.

# Once all the paths are complete, move the view around and save screenshots with Ctrl-s

./build/antpov -f data/seville/ground_and_veg_inner_circular.gltf \
               -c data/seville/Ant06R04-warped.csv.world.csv,data/seville/Ant06R05-warped.csv.world.csv,data/seville/Ant06R06-warped.csv.world.csv,data/seville/Ant06R07-warped.csv.world.csv,data/seville/Ant06ZVF-warped.csv.world.csv -x -R
