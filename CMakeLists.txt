#
# craysim_minimal example CMakeLists.txt file. Use this as a template
# for craysim based simulators.
#

cmake_minimum_required(VERSION 3.28.5 FATAL_ERROR)

project(antpov LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Put binaries directly in the build dir
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

# It appears to be critical to set this for some component of the model. Without it, I see an error like:
# antpov: /home/seb/src/antpov/extern/tinygltf/tiny_gltf.h:347: \
#  const Value &tinygltf::Value::Get(const std::string &) const: Assertion `IsObject()' failed.
# Aborted (core dumped)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release"
    CACHE STRING "Choose build type from: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif(NOT CMAKE_BUILD_TYPE)

# The file FindOptiX.cmake is provided by craysim
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/extern/craysim/cmake" ${CMAKE_MODULE_PATH})

# Path to the NVidia OptiX SDK directory e.g. ~/src/NVIDIA-OptiX-SDK-8.0.0-linux64-x86_64
set(OptiX_INSTALL_DIR "${PROJECT_SOURCE_DIR}/../" CACHE PATH "Path to OptiX installed location.")

# Find the libraries which will be needed in addition to compound-ray (libEyeRenderer3.so)
find_package(glfw3 3.2...3.4 REQUIRED)
find_package(OpenGL)
find_package(OpenMP) # (OpenMP may be used to build navmesh)
find_package(Freetype REQUIRED)
find_package(HDF5 REQUIRED)
find_package(OptiX REQUIRED)
find_package(compound-ray)

# Where is mathplot? (allows headers and modules to be found)
set(MPLOT_INCLUDE_PATH "${PROJECT_SOURCE_DIR}/extern/mathplot" CACHE PATH "The path to mathplot")

# Assume a gcc compiler (or a gcc mimic like Clang). OpenGL 4.3 required so no Apple.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3 -Wno-unused-result -Wno-unknown-pragmas -Wfatal-errors")
# Tell clang to be quiet about brace initialisers:
if(CMAKE_CXX_COMPILER_ID MATCHES Clang)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-missing-braces")
endif()

if(OpenMP_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# Additional GL compiler flags.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DGL3_PROTOTYPES -DGL_GLEXT_PROTOTYPES")

# Tell the program where the morph fonts are, to compile them into the binary
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DMPLOT_FONTS_DIR=\"\\\"${MPLOT_INCLUDE_PATH}/fonts\\\"\"")

if(HDF5_FOUND)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${HDF5_DEFINITIONS}")
endif()

# find_package(glfw3...) does not set any 'GLFW_LIBS' variable, so set one manually
if(UNIX)
  set(GLFW_LIB_NAME glfw)
else()
  set(GLFW_LIB_NAME glfw3)
endif()

set(MPLOT_LIBS_CORE ${HDF5_C_LIBRARIES})
set(MPLOT_LIBS_GL ${GLFW_LIB_NAME} Freetype::Freetype)

#
# Include paths for module compilation
#

# mathplot includes
include_directories(
  ${HDF5_INCLUDE_DIR}
  ${OPENGL_INCLUDE_DIR} ${FREETYPE_INCLUDE_DIRS} ${MPLOT_INCLUDE_PATH}/mplot/glad
  ${MPLOT_INCLUDE_PATH}
)

# Add the path to the OptiX headers to our include paths. This is
# required to compile craysim.virtual, which includes compound-ray
# headers, which refer to some OptiX data structures
include_directories("${OptiX_INCLUDE}")

# allows #include <tinygltf/tiny_gltf.h> (an oces dependency)
include_directories ("${PROJECT_SOURCE_DIR}/extern")

#
# Define lists of module files to build
#

set(EXTERN_DIR ${PROJECT_SOURCE_DIR}/extern)
set(OCES_MODULE_PATH ${EXTERN_DIR}/oces_viewer)
set(CRAYSIM_MODULE_PATH ${EXTERN_DIR}/craysim)
set(MATHS_MODULE_PATH ${EXTERN_DIR}/maths)

# Modules in this repo
set(MY_MODULES
  ${CRAYSIM_MODULE_PATH}/modules/tk_spline.cppm           # GPL v2
  ${CRAYSIM_MODULE_PATH}/modules/craysim_random_walk.cppm # Uses tk_spline, GPL v2
  ${CRAYSIM_MODULE_PATH}/modules/craysim_visual.cppm      # Uses random_walk, GPL v2
  ${CRAYSIM_MODULE_PATH}/modules/craysim_antbody.cppm
  ${PROJECT_SOURCE_DIR}/modules/antpov_helpers.cppm
)
# The OCES reader code
set(OCES_MODULES ${OCES_MODULE_PATH}/oces/reader)
# sebsjames/maths
set(MATHS_CORE_MODULES
  ${MATHS_MODULE_PATH}/sm/mathconst.cppm
  ${MATHS_MODULE_PATH}/sm/config.cppm
  ${MATHS_MODULE_PATH}/sm/constexpr_math.cppm
  ${MATHS_MODULE_PATH}/sm/polysolve.cppm
  ${MATHS_MODULE_PATH}/sm/bessel_i0.cppm
  ${MATHS_MODULE_PATH}/sm/trait_tests.cppm
  ${MATHS_MODULE_PATH}/sm/range.cppm
  ${MATHS_MODULE_PATH}/sm/random.cppm
  ${MATHS_MODULE_PATH}/sm/vec.cppm
  ${MATHS_MODULE_PATH}/sm/vvec.cppm
  ${MATHS_MODULE_PATH}/sm/quaternion.cppm
  ${MATHS_MODULE_PATH}/sm/mat.cppm
)
set(MATHS_MATHPLOT_MODULES
  ${MATHS_MODULE_PATH}/sm/scale.cppm
  ${MATHS_MODULE_PATH}/sm/hdfdata.cppm
  ${MATHS_MODULE_PATH}/sm/winder.cppm
  ${MATHS_MODULE_PATH}/sm/grid.cppm
  ${MATHS_MODULE_PATH}/sm/algo.cppm
  ${MATHS_MODULE_PATH}/sm/histo.cppm
  ${MATHS_MODULE_PATH}/sm/geometry.cppm
  ${MATHS_MODULE_PATH}/sm/geometry_polyhedra.cppm
  ${MATHS_MODULE_PATH}/sm/flags.cppm
  ${MATHS_MODULE_PATH}/sm/centroid.cppm
  ${MATHS_MODULE_PATH}/sm/util.cppm
  ${MATHS_MODULE_PATH}/sm/base64.cppm
  ${MATHS_MODULE_PATH}/sm/crc32.cppm
)
# sebsjames/mathplot
set(MATHPLOT_CORE_MODULES
  ${MPLOT_INCLUDE_PATH}/mplot/fps/profiler.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VisualOwnable.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/Visual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VisualGlfw.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VisualFace.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/TextFeatures.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/TextGeometry.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VisualResources.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VisualCommon.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VisualFont.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VisualTextModel.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VisualModel.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VisualDataModel.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/NavMesh.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/ColourMap.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/colourmaps_cet.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/colourmaps_crameri.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/ColourMap_Lists.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/CoordArrows.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/win_t.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/colour.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/tools.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/unicode.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/loadpng.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/gl/version.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/gl/util_mx.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/graphing.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/graphstyles.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/DatasetStyle.cppm
)
set(ANTPOV_VM_MODULES
  ${MPLOT_INCLUDE_PATH}/mplot/GraphVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/GridVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/RodVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VectorVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VerticesVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/SphereVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/NormalsVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/interop.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/Ommatidium.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/EyeVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/InstancedScatterVisual.cppm
)
set(FIDUCIALS_VM_MODULES
  ${MPLOT_INCLUDE_PATH}/mplot/RhomboVisual.cppm
)

# Compile stuff in subdirectories
add_subdirectory(src)

# My simple eye-creating code
add_subdirectory(simple_eyes)

# Debugging
option(DEBUG_VARIABLES OFF)
if(DEBUG_VARIABLES)
  get_cmake_property(_variableNames VARIABLES)
  foreach(_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
endif(DEBUG_VARIABLES)
