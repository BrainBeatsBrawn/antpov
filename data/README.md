# Eye, scene and route data

This directory contains the data required to run the simulations.

## Scene models

Files with a .gltf suffix are (mostly) glTF encoded 3D models.

There may also be navmesh files which are binary navigation meshes that match up to a glTF file.
These are named with a number that links the file to its model (the filename contains a hash of the model's vertices).
`antpov` will generate the navmesh files if they are not present.
As this can take a long time for larger models, the navmesh is read from the file if it is present when antpov starts.

## Eye files

Found in the **eyes/** subdirectory

These are CSV data files (read by compound-ray) that specify ommatidial position, direction, acceptance angle and focal point offset for a compound-ray eye.

There may also be an associated Open Compound Eye Standard glTF file that specifies the head model.
A draft of Open Compound Eye Standard is described in Blayze's' [thesis](https://etheses.whiterose.ac.uk/id/eprint/37113/) and there is a [reader and viewer](https://github.com/BrainBeatsBrawn/oces_viewer). 

## Route data

Seville ant route data is found in the **seville/** subdirectory.

The original route data from the [CATER project](https://cater.cvmls.org/) is in **seville/orig_paths**.

CSV files specifying an ant's 2D position in the Seville model (ground_and_veg_inner_circular.gltf) all have the suffix `-warped.csv.world.csv`.
These have been converted from the original paths and warped to match the 3D scanned model, as described in the paper.


## camera positioning files

These are JSON files with 'film directions' for movie making.
These are written in a new mathplot format for scripting scene views, which was used by the authors to generate visual renderings of the ant's movements.