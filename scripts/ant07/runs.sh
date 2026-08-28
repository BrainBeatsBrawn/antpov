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

# Ant 07 Available: R1,R3,R6,7,8,9,ZVF

./build/antpov -f data/seville/ground_and_veg_inner_circular.gltf \
               -c \
data/seville/mosaic-warped/mosaic-world/Ant07R08-warped.csv.world.csv,\
data/seville/mosaic-warped/mosaic-world/Ant07R09-warped.csv.world.csv,\
data/seville/mosaic-warped/mosaic-world/Ant07ZVF-warped.csv.world.csv\
 -x -R
