#!/bin/bash

# Ant03R01 for Fig. 3C

./build/antpov -f data/seville/ground_and_veg_inner_circular.gltf \
               -c data/seville/Ant03R01-warped.csv.world.csv \
               -j Ant03R01-warped.csv.world.topdown.json -x -m
