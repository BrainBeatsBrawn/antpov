#!/bin/bash

# Command to plot four paths; Rmax for each of Ants 3,6,11,12.
#
# Colour code:

# Ant 12 Orange for routes, black for ZVF

# Once all the paths are complete, move the view around and save screenshots with Ctrl-S

./build/antpov -f data/seville/ground_and_veg_inner_circular.gltf \
               -c data/seville/Ant12R01-warped.csv.world.csv,data/seville/Ant12R02-warped.csv.world.csv,data/seville/Ant12ZVF-warped.csv.world.csv -x
