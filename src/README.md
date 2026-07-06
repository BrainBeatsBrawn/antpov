# Source code

This directory holds the source for the programs in this repository.

## antpov.cpp

The main **antpov** program. This is very much a research code script! It uses a a `craysim_visual<>` to load and render the ant within the scene, and some `mplot::Visual` windows to render the ant's eyes separately from the scene.

Some additional code is in the C++ module file **antpov_helpers.cppm**.

Example invocations:

```bash
./build/antpov -f data/natural_env.gltf
```

```bash
./build/antpov -f data/seville/ground_and_veg_inner_circular.gltf \
               -c data/seville/Ant03R01-warped.csv.world.csv
```

## cater_graph.cpp

Graphs one of the original CATER paths.

Example invocations:

```bash
./build/cater_graph data/seville/orig_paths/Ant11R01.csv
```

## cater_paths.cpp

Draws graphs all of the original CATER paths. Invocation:

```bash
./build/cater_paths
```

## cater_sel_paths.cpp

Reproduces the paths shown in Figure 3 A of [CATER: Combined Animal Tracking & Environment Reconstruction](https://doi.org/10.1126/sciadv.adg2094).

Invocation:

```bash
./build/cater_sel_paths
```

## dirns.cpp

A program used to test the method of inferring the ant direction (or our lack of knowledge of its direction) from the sequence of positions.
The directions are inferred, then plotted (zoom in to see that each point is a tiny arrow).

Example invocation:

```bash
./build/dirns data/seville/Ant03R02-warped.csv.world.csv
```

## fiducials.cpp

This was a program to generate a glTF file containing some fiducial markers, used while developing antpov.