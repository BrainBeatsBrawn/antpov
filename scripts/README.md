# Scripts to reproduce paper figures

This folder contains scripts that make it possible to reproduce the results from the paper.

## Pre-requisites

Before running the scripts, you will need to download the Seville environment glTF file, which is too large to distribute in this repository.

Download from our Dataset:

[Dataset: Reconstructing the visual histories of desert ants](https://doi.org/10.15131/shef.data.32860832),

Enter the folder **3. Reconstructing Ant Views**  and download **Seville_scanned_model.zip**.

Unpack the zip file to obtain both the glTF (**ground_and_veg_inner_circular.gltf**) and the associated NavMesh file (**navmesh_4420489099394405100**).

Place these files in antpov/data/seville/

You will also need to have compiled the software in a directory antpov/build, after following the build instructions in the [top-level readme](https://github.com/BrainBeatsBrawn/antpov/blob/main/README.md).

## ant03

This sub-directory contains scripts that run antpov to reproduce Ant 03's run 01, which is shown in Figure 3.
Run any of the scripts from the base antpov:

```
./scripts/ant03/r01_various_cam_angles.sh 
```

## ant12

Ant 12's run 01, 02 and Zero Vector run. These are the runs shown in Figure 4. 

```
./scripts/ant12/r01_r02_zvf.sh
```

## four_paths

This script runs four different Ant runs in sequence to reproduce the model from which Figure 1D is a screenshot.

It plots the last forage for each of Ants 3, 6, 11 and 12.

```
./scripts/four_paths/four_paths.sh 
```

## Making movies

If you want to make a movie of any of these runs, edit the script and add `-x -m` to the antpov command invocation.
