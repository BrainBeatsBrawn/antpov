# eye3D (with compound-ray)

This is a template repository for building a program that uses compound-ray

This simple repository builds one program, **eye3d**, which opens a
glTF-encoded 3D environment containing a compound eye camera and
renders the view both in a 2D representation (from compound-ray) and a
3D view, using a morphologica VisualModel called
comray::CompoundEyeVisual.

Before compiling c_ray_eye3d, obtain and compile NVidia Optix 8.0 and obtain,
compile and `make install` compound-ray from:

https://github.com/optseb/compound-ray (branch **brain_render** for now)

To compile eye3d, first clone sebsjames/mathplot, sebsjames/maths and Opteran's mathplot
extensions into the base of this repo:

```bash
cd c_ray_eye3d
git clone git@github.com:sebsjames/maths
git clone git@github.com:sebsjames/mathplot
git clone git@github.com:Opteran/mplotext
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
./build/bin/eye3d -f ./data/natural_env.gltf
```

## To use this repo as a template

Simply make a copy of the repo (or use it as a template on github) and
then replace the source file src/eye3d.cpp with your program.

Author: Seb James
Date: May 2024
