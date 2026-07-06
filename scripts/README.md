# Scripts to reproduce paper figures

This folder contains scripts that make it possible to reproduce the results from the paper.

## Pre-requisites

Before running the scripts, you will need to download the Seville environment glTF file, which is too large to distribute in this repository.

Download the following files and place them in antpov/data/seville/

(Correct URLs to be added)

https://path.to/ground_and_veg_inner_circular.gltf

https://path.to/navmesh_4420489099394405100

or

https://path.to/antpov_seville_data.zip

You will also need to have compiled the software in a directory antpov/build, after following the build instructions in the [top-level readme](https://github.com/BrainBeatsBrawn/antpov/blob/main/README.md).

## ant03

This sub-directory contains scripts that run antpov to reproduce Ant 03's run 01, which is shown in Figure 3.
Run any of the scripts from the base antpov:

```
./scripts/ant03/r01_various_cam_angles.sh 
```

## ant12

Ant 12's run 01, 02 and Zero Vector run. These are the runs shown in Figure 4. 

## four_paths

This script runs four different Ant runs in sequence to reproduce the model from which Figure 1D is a screenshot.

## Making movies

If you want to make a movie of any of these runs, edit the script and add `-x -m` to the antpov command invocation.
