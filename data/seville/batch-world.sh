#!/bin/bash

for i in *-warped.csv; do
    ./to-world-coords.py $i "-14,-14" "14,14" > $i.world.csv
done
