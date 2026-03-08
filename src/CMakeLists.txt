# All #includes in test programs have to be #include <mplot/header.h>
include_directories(BEFORE ${PROJECT_SOURCE_DIR})

# Each C++ module file has to be listed here (as they don't have a .cppm or .ixx file suffix)
set_source_files_properties (
  ${MPLOT_INCLUDE_PATH}/maths/sm/range
  ${MPLOT_INCLUDE_PATH}/maths/sm/random
  ${MPLOT_INCLUDE_PATH}/maths/sm/vec
  ${MPLOT_INCLUDE_PATH}/maths/sm/vvec
  ${MPLOT_INCLUDE_PATH}/maths/sm/scale
  ${MPLOT_INCLUDE_PATH}/maths/sm/hdfdata
  ${MPLOT_INCLUDE_PATH}/maths/sm/quaternion
  ${MPLOT_INCLUDE_PATH}/maths/sm/mat
  ${MPLOT_INCLUDE_PATH}/maths/sm/hex
  ${MPLOT_INCLUDE_PATH}/maths/sm/bezcoord
  ${MPLOT_INCLUDE_PATH}/maths/sm/bezcurve
  ${MPLOT_INCLUDE_PATH}/maths/sm/winder
  ${MPLOT_INCLUDE_PATH}/maths/sm/bootstrap
  ${MPLOT_INCLUDE_PATH}/maths/sm/grid
  ${MPLOT_INCLUDE_PATH}/maths/sm/algo
  ${MPLOT_INCLUDE_PATH}/maths/sm/nm_simplex
  ${MPLOT_INCLUDE_PATH}/maths/sm/histo
  ${MPLOT_INCLUDE_PATH}/maths/sm/boxfilter
  ${MPLOT_INCLUDE_PATH}/maths/sm/geometry
  ${MPLOT_INCLUDE_PATH}/maths/sm/geometry_polyhedra
  ${MPLOT_INCLUDE_PATH}/maths/sm/flags
  ${MPLOT_INCLUDE_PATH}/maths/sm/centroid
  ${MPLOT_INCLUDE_PATH}/maths/sm/util
  ${MPLOT_INCLUDE_PATH}/maths/sm/base64
  ${MPLOT_INCLUDE_PATH}/maths/sm/crc32
  ${MPLOT_INCLUDE_PATH}/mplot/VisualOwnable.h
  ${MPLOT_INCLUDE_PATH}/mplot/Visual.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualGlfw.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualFace.h
  ${MPLOT_INCLUDE_PATH}/mplot/TextFeatures.h
  ${MPLOT_INCLUDE_PATH}/mplot/TextGeometry.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualResources.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualCommon.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualFont.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualTextModel.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualModel.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualDataModel.h
  ${MPLOT_INCLUDE_PATH}/mplot/NavMesh.h
  ${MPLOT_INCLUDE_PATH}/mplot/ColourMap.h
  ${MPLOT_INCLUDE_PATH}/mplot/CoordArrows.h
  ${MPLOT_INCLUDE_PATH}/mplot/win_t.h
  ${MPLOT_INCLUDE_PATH}/mplot/colour.h
  ${MPLOT_INCLUDE_PATH}/mplot/tools.h
  ${MPLOT_INCLUDE_PATH}/mplot/fps/profiler.h
  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/interop.h
  ${MPLOT_INCLUDE_PATH}/mplot/gl/version.h
  ${MPLOT_INCLUDE_PATH}/mplot/gl/util_mx.h

  ${MPLOT_INCLUDE_PATH}/mplot/graphing.h
  ${MPLOT_INCLUDE_PATH}/mplot/graphstyles.h
  ${MPLOT_INCLUDE_PATH}/mplot/DatasetStyle.h
  ${MPLOT_INCLUDE_PATH}/mplot/GraphVisual.h

  ${MPLOT_INCLUDE_PATH}/mplot/GridVisual.h

  ${MPLOT_INCLUDE_PATH}/mplot/RodVisual.h

  ${MPLOT_INCLUDE_PATH}/mplot/VectorVisual.h

  ${MPLOT_INCLUDE_PATH}/mplot/SphereVisual.h

  ${MPLOT_INCLUDE_PATH}/mplot/NormalsVisual.h

  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/EyeVisual.h

  ${MPLOT_INCLUDE_PATH}/mplot/InstancedScatterVisual.h

  PROPERTIES LANGUAGE CXX
)

include_directories (BEFORE "${PROJECT_SOURCE_DIR}/extern/oces_viewer")
include_directories (BEFORE "${PROJECT_SOURCE_DIR}/extern/oces_viewer/extern")

# Build glad library here
include_directories(BEFORE ${MPLOT_INCLUDE_PATH}/mplot/glad)
add_library(glad STATIC ${MPLOT_INCLUDE_PATH}/mplot/glad/gl.c)

# Build mathplot fonts data
add_library(mplot_fontdata ${MPLOT_INCLUDE_PATH}/mplot/VisualFaceAsm.cpp)

# The OptiX way to create an executable (Do I have to hack this for modules?)
OPTIX_add_sample_executable (antpov target_name antpov.cpp OPTIONS -rdc true)

target_sources(${target_name} PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES
  ${MPLOT_INCLUDE_PATH}/mplot/tools.h
  ${MPLOT_INCLUDE_PATH}/mplot/gl/version.h
  ${MPLOT_INCLUDE_PATH}/mplot/gl/util_mx.h
  ${MPLOT_INCLUDE_PATH}/mplot/colour.h
  ${MPLOT_INCLUDE_PATH}/mplot/win_t.h
  ${MPLOT_INCLUDE_PATH}/mplot/CoordArrows.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualOwnable.h
  ${MPLOT_INCLUDE_PATH}/mplot/Visual.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualGlfw.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualFace.h
  ${MPLOT_INCLUDE_PATH}/mplot/TextFeatures.h
  ${MPLOT_INCLUDE_PATH}/mplot/TextGeometry.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualResources.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualCommon.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualFont.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualTextModel.h
  ${MPLOT_INCLUDE_PATH}/mplot/VisualModel.h
  ${MPLOT_INCLUDE_PATH}/mplot/NavMesh.h
  ${MPLOT_INCLUDE_PATH}/mplot/RodVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/NormalsVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/fps/profiler.h
  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/interop.h
  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/EyeVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/ColourMap.h
  ${MPLOT_INCLUDE_PATH}/maths/sm/util
  ${MPLOT_INCLUDE_PATH}/maths/sm/base64
  ${MPLOT_INCLUDE_PATH}/maths/sm/crc32
  ${MPLOT_INCLUDE_PATH}/maths/sm/flags
  ${MPLOT_INCLUDE_PATH}/maths/sm/algo
  ${MPLOT_INCLUDE_PATH}/maths/sm/range
  ${MPLOT_INCLUDE_PATH}/maths/sm/random
  ${MPLOT_INCLUDE_PATH}/maths/sm/vec
  ${MPLOT_INCLUDE_PATH}/maths/sm/quaternion
  ${MPLOT_INCLUDE_PATH}/maths/sm/mat
  ${MPLOT_INCLUDE_PATH}/maths/sm/geometry
  ${MPLOT_INCLUDE_PATH}/maths/sm/geometry_polyhedra
  ${MPLOT_INCLUDE_PATH}/maths/sm/vvec)

# Link CUDA, mathplot dependencies and libEyeRenderer3.so omit: ${CUDA_LIBRARIES}
target_link_libraries(${target_name} ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} compound-ray::EyeRenderer3 glad mplot_fontdata)

add_executable (fiducials fiducials.cpp)
target_link_libraries(fiducials ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glfw)
