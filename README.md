# Ant POV sim (made with mathplot, compound-ray, craysim and oces_viewer)

A program to show an ant's point of view in a model environment

**antpov** opens a
glTF-encoded 3D environment containing an ant-form compound eye camera and
renders the ant's view.

## Prerequisites

Before compiling antpov, obtain and compile NVidia Optix 8.0 and obtain,
compile and `make install` compound-ray from:

https://github.com/sebsjames/compound-ray

You will need to use gcc-12 or gcc-11 to compile compound-ray (or possibly some version of clang).

To compile antpov, you need clang-20 and ninja-build:

```bash
sudo apt-install clang-20 clang-tools-20 libc++-20-dev ninja-build
```

You also need cmake version 3.28.5 or higher. Either `apt install cmake`
on Ubuntu 25+ or download and build cmake from the cmake.org download
page (it's an easy, reliable compile).

Also the mathplot dependencies (wrt branch dev/modules)

## Submodules

Antpov uses the submodules [sebsjames/mathplot](https://github.com/sebsjames/mathplot), [sebsjames/maths](https://github.com/sebsjames/maths), [BrainBeatsBrawn/craysim](https://github.com/BrainBeatsBrawn/craysim), [BrainBeatsBrawn/oces_viewer](https://github.com/BrainBeatsBrawn/oces_viewer) and [tinygltf](https://github.com/sebsjames/tinygltf). The program links to Seb's fork of [compound-ray](https://github.com/BrainBeatsBrawn/compound-ray)

## Build

To compile antpov, recursively clone it, or if you already cloned, init/update the submodules

```bash
cd antpov
git submodule init
git submodule update
```

then do a cmake build, passing the cmake variable OptiX_INSTALL_DIR
just as you will have done so for compound-ray:

```bash
mkdir build
cd build
CXX=clang++-20 cmake .. -GNinja -DOptiX_INSTALL_DIR=~/src/NVIDIA-OptiX-SDK-8.0.0-linux64-x86_64
# Optional: -DCMAKE_CXX_FLAGS="-stdlib=libc++"
make
```

Now you can run the program

```bash
./build/antpov -f ./data/natural_env.gltf
```

Author: Seb James
Date: Jan 2026
