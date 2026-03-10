#
# A function used here to generate an Optix + C++ modules executable
#
function(add_modules_exe target_name_base target_name_var)

  set( target_name ${target_name_base} )
  set( ${target_name_var} ${target_name} PARENT_SCOPE )

  # Parse the rest of the args, which are all multiValueArgs
  set(multiValueArgs SOURCES MODULES OPTIONS CMAKE_OPTIONS)
  cmake_parse_arguments(PARSE_ARGV 0 arg
    "${options}" "${oneValueArgs}" "${multiValueArgs}"
  )

  # These calls will group PTX and CUDA files into their own directories in the Visual
  # Studio projects.
  if (NOT CUDA_NVRTC_ENABLED)
    source_group("PTX Files"  REGULAR_EXPRESSION ".+\\.ptx$")
  endif()
  source_group("CUDA Files" REGULAR_EXPRESSION ".+\\.cu$")

  if(CUDA_NVRTC_ENABLED)
    # Isolate OBJ target files. NVCC should only process these files and leave PTX targets for NVRTC
    set(cu_obj_source_files)
    foreach(file ${arg_SOURCES})
      get_source_file_property(_cuda_source_format ${file} CUDA_SOURCE_PROPERTY_FORMAT)
      if(${_cuda_source_format} MATCHES "OBJ")
        list(APPEND cu_obj_source_files ${file})
      endif()
    endforeach()
    # Create the rules to build the OBJ from the CUDA files.
    CUDA_WRAP_SRCS( ${target_name} OBJ generated_files ${cu_obj_source_files} ${arg_CMAKE_OPTIONS} OPTIONS ${arg_OPTIONS} )
  else()
    # Create the rules to build the PTX and OBJ from the CUDA files.
    CUDA_WRAP_SRCS( ${target_name} PTX generated_files ${arg_SOURCES} ${arg_CMAKE_OPTIONS} OPTIONS ${arg_OPTIONS} )
  endif()

  add_executable(${target_name} ${arg_SOURCES} ${generated_files} ${arg_CMAKE_OPTIONS})

  # We'll have to pass in MODULES_FILES or something
  target_sources(${target_name} PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR}
    FILES ${arg_MODULES}
  )

  target_link_libraries(${target_name} Threads::Threads ${GLFW_LIB_NAME} sutil_7_sdk)
  set_target_properties(${target_name} PROPERTIES COMPILE_DEFINITIONS OPTIX_SAMPLE_NAME_DEFINE=${target_name})

  if(UNIX AND NOT APPLE)
    # Force using RPATH instead of RUNPATH on Debian
    target_link_libraries( ${target_name} "-Wl,--disable-new-dtags" )
  endif()

  if(USING_GNU_CXX)
    target_link_libraries( ${target_name} m ) # Explicitly link against math library
  endif()
endfunction()


# All #includes in test programs have to be #include <mplot/header.h>
include_directories(BEFORE ${PROJECT_SOURCE_DIR})

include_directories (BEFORE "${PROJECT_SOURCE_DIR}/extern/oces_viewer")
include_directories (BEFORE "${PROJECT_SOURCE_DIR}/extern/oces_viewer/extern")

#
# Define lists of module files
#
# Modules in this repo
set(MY_MODULES
  ${PROJECT_SOURCE_DIR}/include/spline.hpp
  ${PROJECT_SOURCE_DIR}/include/antpovvisual.h
  ${PROJECT_SOURCE_DIR}/include/AntBodyVisual.h
)
# The OCES reader code
set(OCES_MODULES ${PROJECT_SOURCE_DIR}/extern/oces_viewer/oces/reader)
# sebsjames/maths
set(MATHS_CORE_MODULES
  ${MPLOT_INCLUDE_PATH}/maths/sm/range
  ${MPLOT_INCLUDE_PATH}/maths/sm/random
  ${MPLOT_INCLUDE_PATH}/maths/sm/vec
  ${MPLOT_INCLUDE_PATH}/maths/sm/vvec
  ${MPLOT_INCLUDE_PATH}/maths/sm/quaternion
  ${MPLOT_INCLUDE_PATH}/maths/sm/mat
)
set(MATHS_MATHPLOT_MODULES
  ${MPLOT_INCLUDE_PATH}/maths/sm/scale
  ${MPLOT_INCLUDE_PATH}/maths/sm/hdfdata
  ${MPLOT_INCLUDE_PATH}/maths/sm/winder
  ${MPLOT_INCLUDE_PATH}/maths/sm/grid
  ${MPLOT_INCLUDE_PATH}/maths/sm/algo
  ${MPLOT_INCLUDE_PATH}/maths/sm/histo
  ${MPLOT_INCLUDE_PATH}/maths/sm/geometry
  ${MPLOT_INCLUDE_PATH}/maths/sm/geometry_polyhedra
  ${MPLOT_INCLUDE_PATH}/maths/sm/flags
  ${MPLOT_INCLUDE_PATH}/maths/sm/centroid
  ${MPLOT_INCLUDE_PATH}/maths/sm/util
  ${MPLOT_INCLUDE_PATH}/maths/sm/base64
  ${MPLOT_INCLUDE_PATH}/maths/sm/crc32
)
# sebsjames/mathplot
set(MATHPLOT_CORE_MODULES
  ${MPLOT_INCLUDE_PATH}/mplot/fps/profiler.h
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
  ${MPLOT_INCLUDE_PATH}/mplot/colourmaps_cet.h
  ${MPLOT_INCLUDE_PATH}/mplot/colourmaps_crameri.h
  ${MPLOT_INCLUDE_PATH}/mplot/ColourMap_Lists.h
  ${MPLOT_INCLUDE_PATH}/mplot/CoordArrows.h
  ${MPLOT_INCLUDE_PATH}/mplot/win_t.h
  ${MPLOT_INCLUDE_PATH}/mplot/colour.h
  ${MPLOT_INCLUDE_PATH}/mplot/tools.h
  ${MPLOT_INCLUDE_PATH}/mplot/gl/version.h
  ${MPLOT_INCLUDE_PATH}/mplot/gl/util_mx.h
  ${MPLOT_INCLUDE_PATH}/mplot/graphing.h
  ${MPLOT_INCLUDE_PATH}/mplot/graphstyles.h
  ${MPLOT_INCLUDE_PATH}/mplot/DatasetStyle.h
)
set(ANTPOV_VM_MODULES
  ${MPLOT_INCLUDE_PATH}/mplot/GraphVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/GridVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/RodVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/VectorVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/VerticesVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/SphereVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/NormalsVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/interop.h
  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/EyeVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/InstancedScatterVisual.h
)
set(FIDUCIALS_VM_MODULES
  ${MPLOT_INCLUDE_PATH}/mplot/RhomboVisual.h
)

# Each C++ module file has to be listed here (as they don't have a .cppm or .ixx file suffix)
set_source_files_properties (
  ${MY_MODULES}
  ${OCES_MODULES}
  ${MATHS_CORE_MODULES}
  ${MATHS_MATHPLOT_MODULES}
  ${MATHPLOT_CORE_MODULES}
  ${ANTPOV_VM_MODULES}
  ${FIDUCIALS_VM_MODULES}
  PROPERTIES LANGUAGE CXX
)

# Build glad library here
include_directories(BEFORE ${MPLOT_INCLUDE_PATH}/mplot/glad)
add_library(glad STATIC ${MPLOT_INCLUDE_PATH}/mplot/glad/gl.c)

# Build mathplot fonts data
add_library(mplot_fontdata STATIC ${MPLOT_INCLUDE_PATH}/mplot/VisualFaceAsm.cpp)

# A special exe to make fiducials for the Seville environment
add_executable (fiducials fiducials.cpp)
set(FIDUCIALS_MODULES ${MATHS_CORE_MODULES} ${MATHS_MATHPLOT_MODULES} ${MATHPLOT_CORE_MODULES} ${FIDUCIALS_VM_MODULES})
target_sources(fiducials PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR} FILES ${FIDUCIALS_MODULES})
target_link_libraries(fiducials ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glfw glad mplot_fontdata)

# The main ant simulator
set(ANTPOV_MODULES ${MY_MODULES} ${OCES_MODULES} ${MATHS_CORE_MODULES} ${MATHS_MATHPLOT_MODULES} ${MATHPLOT_CORE_MODULES} ${ANTPOV_VM_MODULES})
# A hacked OptiX function to generate modules-happy exe
add_modules_exe (antpov target_name SOURCES antpov.cpp MODULES ${ANTPOV_MODULES} OPTIONS -rdc true)
# Link CUDA, mathplot dependencies and libEyeRenderer3.so omit: ${CUDA_LIBRARIES}
target_link_libraries(antpov ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} compound-ray::EyeRenderer3 glad mplot_fontdata)
