#!/bin/bash

# Ant03R01 for local topdown video

./build/antpov -f data/seville/ground_and_veg_inner_circular.gltf \
               -c data/seville/Ant03R01-warped.csv.world.csv \
               -j data/seville/Ant03R01-warped.csv.world_topdown.json \
               -x -m
