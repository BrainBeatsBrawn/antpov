These files were created from -warped.csv files along with corresponding _labels.csv files for each Ant track. The script to-world-coords.py was used to do this.

I used a script to do a batch of them:

#!/bin/bash

for i in *-warped.csv; do
    ./to-world-coords.py $i "-14,-14" "14,14" > $i.world.csv
done

The *.world.csv files were used to create *.world.csv.3d.csv files, which are 3D locations obtained by fitting an ant agent to the 2D locations in *.world.csv.