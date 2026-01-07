# Ant POV sim (made with mathplot, compound-ray and oces_viewer)

A program to show an ant's point of view in a model environment

**antpov** opens a
glTF-encoded 3D environment containing an ant-form compound eye camera and
renders the ant's view.

Before compiling antpov, obtain and compile NVidia Optix 8.0 and obtain,
compile and `make install` compound-ray from:

https://github.com/sebsjames/compound-ray

To compile antpov, first recursively clone sebsjames/mathplot and syoyo/tinygltf into the extern folder of this repo:

```bash
cd antpov
mkdir extern
cd extern
git clone git@github.com:sebsjames/mathplot --recurse-submodules
# Now also OCES viewer code and supporting tinygltf:
git clone git@github.com:sebsjames/oces_viewer --recurse-submodules
# git clone git@github.com:syoyo/tinygltf
cd ..
```

then do a cmake build, passing the cmake variable OptiX_INSTALL_DIR
just as you will have done so for compound-ray:

```bash
mkdir build
cd build
cmake -DOptiX_INSTALL_DIR=~/src/NVIDIA-OptiX-SDK-8.0.0-linux64-x86_64 ..
make
```

Now you can run the program

```bash
./build/bin/antpov -f ./data/natural_env.gltf
```

Author: Seb James
Date: Jan 2026
