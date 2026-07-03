# Ant POV simulator

**antpov** is a program to show an ant's point of view in a model environment.

It opens a
glTF-encoded 3D environment containing an ant-form compound eye camera and
renders the ant's view.

Figures from the associated paper (to be linked here) give you a flavour of what the software was designed to do.

## Digital twins present new opportunities and challenges for neuroethology research

![Figure 4A and 4B from the paper](https://github.com/BrainBeatsBrawn/antpov/blob/main/images/ValueOfDigitalTwin.png?raw=true)

**A.** Left: shows a visualisation of a simulated ant (bottom left of image) within a digital twin.  Top right: render of a standard panoramic view (360deg by +/-45deg around the horizon) as often used to simulate insect vision.
Bottom right: render of the biologically constrained eyes outputs.
**B.** Visualisation of ant as it crested a low lying plant.  The insert shows the views rendered at the corresponding locations and pose.

## Reconstructing views from the foraging life of a *C. velox* forager

![Figure 3 from the paper](https://github.com/BrainBeatsBrawn/antpov/blob/main/images/AntLivesExample.png?raw=true)

**A.** The background image shows a frame from the video dataset as Ant 3 left the nest for the first time.
The ant is outlined by a white box; an ant icon indicates her orientation.
The corresponding position in the digital twin (C) is shown by the white line.
The insert shows a reconstruction of the visual input Ant 3 will have experienced at this point in her life.
**B.** A reconstruction of the view experienced by Ant 3 as she collected cookie from the feeder location for the first time.
**C.** The complete first foraging journey of Ant 3 is plotted in the digital twin.
*N* shows the nest location, while F shows the location that the ant was rewarded with cookie.
The foraging path is labelled *Out*, while the homeward path is labelled *In*.}

# Build Ant POV

## Hardware requirements

You will need an NVIDIA GPU and an Intel/AMD computer (running a Linux OS).

You will need about 16 GB of free storage to install the compilers and build tools and to complete the build process.
My freshly installed and updated system started with 9.5 GB of storage used.
The package managed dependencies added 9 GB; CMake and compound-ray was 2 GB, the antpov repository (and its submodules) another 1 GB and finally the build of antpov consumed another 4 GB.

## Building on Ubuntu 24.04

Here's how to build and run antpov on an Ubuntu 24.04 system.
It was verified on a cleanly installed system which had been fully upgraded to Ubuntu 24.04.4 LTS.

## Graphics driver

Use the Ubuntu *Additional Driver* GUI to install an NVIDIA graphics driver that is compatible with your NVIDIA GPU.
If you chose to enable "Third party drivers" when you installed Ubuntu, that should have done the job.
You can verify by opening the NVIDIA Settings program and looking at 'System Information'.
You should see an 'NVIDIA Driver Version' field.
If it's not there, then you probably don't have the NVIDIA driver installed.
I've had successful builds on Ubuntu 24 with both the version 535 driver and the version 580 driver.

## Install package managed dependencies

The following packages are required to build antpov:

```bash
sudo apt install build-essential git \
                 gcc-12 g++-12 \
                 nvidia-cuda-toolkit \
                 clang-20 clang-tools-20 libc++-20-dev ninja-build \
                 freeglut3-dev libglu1-mesa-dev libxmu-dev \
                 libxi-dev libglfw3-dev libfreetype-dev libhdf5-dev
```

* GCC 12 and gmake (from build-essential) is used to compile compound-ray.
* Compound-ray also needs nvidia-cuda-toolkit, which installs the NVIDIA GPU compiler nvcc.
* Clang-20 and ninja are used to compile antpov.
* The libraries freeglut3-dev to libhdf5-dev are required by craysim/mathplot for OpenGL visualizations.

## Compile and install cmake

On Ubuntu 24.04, the packaged CMake is slightly too old for the C++ modules build of antpov, so we compile and install the latest CMake from https://cmake.org/download/
At the time of writing this is version 4.3.4.
CMake versions as old as 3.29 and more recent than 4.3.4 should work too, so if your CMake is already within this range, you can skip this step.

```bash
cd ~/Downloads
wget https://github.com/Kitware/CMake/releases/download/v4.3.4/cmake-4.3.4.tar.gz
mkdir -p ~/src
cd ~/src
tar xvf ~/Downloads/cmake-4.3.4.tar.gz
cd cmake-4.3.4
./bootstrap --parallel=4 # If you have 4 cores
make -j4
sudo make install
```

You will now have **/usr/local/bin/cmake** which will be used in preference to the package-managed cmake in /usr/bin. Verify *in a new terminal*:

```bash
seb@ubu24-vm1:~$ which cmake
/usr/local/bin/cmake
seb@ubu24-vm1:~$ cmake --version
cmake version 4.3.4

CMake suite maintained and supported by Kitware (kitware.com/cmake).
seb@ubu24-vm1:~$
```

## Build compound-ray

Compound-ray is a library of code that performs ray casting to compute the colour detected by each ommatidium in an insect compound eye within a model environment.

It uses NVIDIA OptiX to achieve the ray casting on an NVIDIA GPU.

Compound-ray was written by Blayze Millward ([original code](https://github.com/BrainsOnBoard/compound-ray)). A modified version is used here: https://github.com/BrainBeatsBrawn/compound-ray

### NVIDIA OptiX SDK

You will need to register a developer account with NVIDIA and download the [NVIDIA OptiX](https://developer.nvidia.com/rtx/ray-tracing/optix) SDK, version 8.0 from the [Legacy Downloads page](https://developer.nvidia.com/designworks/optix/downloads/legacy).

After downloading, you should have obtained the installer shellscript file **NVIDIA-OptiX-SDK-8.0.0-linux64-x86_64.sh**. We'll assume it's in ~/Downloads/NVIDIA-OptiX-SDK-8.0.0-linux64-x86_64.sh.

Unpack the file into ~/src:

```bash
mkdir -p ~/src
cd src
bash ~/Downloads/NVIDIA-OptiX-SDK-8.0.0-linux64-x86_64.sh
```

Page down to accept the licence agreement, and then you should be prompted to install in the default location in src/.
Accept the default.
You should now have a directory **~/src/NVIDIA-OptiX-SDK-8.0.0-linux64-x86_64** containing the NVIDIA OptiX SDK.

### Temporarily switch your default compiler to GCC 12

With some build systems, you can specify the compiler version with the environment variables `CC` and `CXX`. For some reason, this does not work here, and instead we have to use the `update-alternatives` system (a Debian/Ubuntu thing) to switch our default compiler from GCC 13 to the OptiX 8.0-compatible GCC 12.

```bash
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 12 --slave /usr/bin/g++ g++ /usr/bin/g++-12 --slave /usr/bin/gcov gcov /usr/bin/gcov-12
```

Verify that g++ is version 12:

```bash
seb@ubu24-vm1:~$ g++ --version
g++ (Ubuntu 12.4.0-2ubuntu1~24.04.1) 12.4.0
Copyright (C) 2022 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
```

### Obtain and compile compound-ray

```bash
cd ~/src
git clone https://github.com/BrainBeatsBrawn/compound-ray
cd compound-ray/build
cmake .. -DOptiX_INSTALL_DIR=~/src/NVIDIA-OptiX-SDK-8.0.0-linux64-x86_64
make
sudo make install # Installs in /usr/local
```

## Build antpov


To compile antpov, clone it then init/update the submodules.

Antpov uses the submodules [sebsjames/mathplot](https://github.com/sebsjames/mathplot), [sebsjames/maths](https://github.com/sebsjames/maths), [BrainBeatsBrawn/craysim](https://github.com/BrainBeatsBrawn/craysim), [BrainBeatsBrawn/oces_viewer](https://github.com/BrainBeatsBrawn/oces_viewer) and [tinygltf](https://github.com/sebsjames/tinygltf).
The program links to Seb's fork of [compound-ray](https://github.com/BrainBeatsBrawn/compound-ray)


```bash
cd ~/src
git clone https://github.com/BrainBeatsBrawn/antpov
cd antpov
git submodule init
git submodule update
```

Now you do a CMake style build, passing the variable `OptiX_INSTALL_DIR`
just as you did for compound-ray and specifying that clang++-20 and ninja should be used to compile:

```bash
mkdir build
cd build
CC=clang-20 CXX=clang++-20 cmake .. -GNinja -DOptiX_INSTALL_DIR=~/src/NVIDIA-OptiX-SDK-8.0.0-linux64-x86_64
ninja
```

That's it. Test by running:

```bash
cd ~/src/antpov
./build/antpov -f ./data/natural_env.gltf
```

### Switch your compiler back

Optionally, change your system back, so that the gcc and g++ commands involke the OS-default GCC 13.
You can do this by adding another alternative for GCC 13 or you can simply delete the gcc alternative like this:

```bash
sudo update-alternatives --remove-all gcc
```

# Use Ant POV


# Credits

Ant POV was authored by Seb James.
Compound-ray was authored by Blayze Millward, with modifications by Seb James.
Alex Blenkinsop contributed to sebsjames/maths and to craysim.
