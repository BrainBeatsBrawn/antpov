# All #includes in test programs have to be #include <mplot/header.h>
#include_directories(BEFORE ${PROJECT_SOURCE_DIR})
#include_directories (BEFORE "${PROJECT_SOURCE_DIR}/extern/oces_viewer")
#include_directories (BEFORE "${PROJECT_SOURCE_DIR}/extern/oces_viewer/extern")

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
add_executable(fiducials fiducials.cpp)
set(FIDUCIALS_MODULES ${MATHS_CORE_MODULES} ${MATHS_MATHPLOT_MODULES} ${MATHPLOT_CORE_MODULES} ${FIDUCIALS_VM_MODULES})
target_sources(fiducials PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR} FILES ${FIDUCIALS_MODULES})
target_link_libraries(fiducials ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glfw glad mplot_fontdata)

# The main ant simulator
set(ANTPOV_MODULES ${MY_MODULES} ${OCES_MODULES} ${MATHS_CORE_MODULES} ${MATHS_MATHPLOT_MODULES} ${MATHPLOT_CORE_MODULES} ${ANTPOV_VM_MODULES})
add_executable(antpov antpov.cpp)
target_sources(antpov
  PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES ${ANTPOV_MODULES}
)
target_link_libraries(antpov ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} compound-ray::EyeRenderer3 glad mplot_fontdata)
