include_directories (BEFORE "${PROJECT_SOURCE_DIR}/mathplot")

# Build glad library here
include_directories(BEFORE ${MPLOT_INCLUDE_PATH}/mplot/glad)
add_library(glad2 STATIC ${MPLOT_INCLUDE_PATH}/mplot/glad/gl.c)

# Build mathplot fonts data
add_library(mplot_fontdata2 STATIC ${MPLOT_INCLUDE_PATH}/mplot/VisualFaceAsm.cpp)

add_executable (ant_eyes ant_eyes.cpp)

# Mathplot and craysim modules are used
set(ANTEYES_MODULES
  ${MPLOT_MATHS_CORE_MODULES}
  ${MPLOT_MATHS_VISUALMODEL_MODULES}
  ${MPLOT_CORE_MODULES}
  ${CRAYSIM_MATHS_DOUBLEHEX_MODULES}
  ${CRAYSIM_MATHPLOT_DOUBLEHEX_MODULES}
  ${CRAYSIM_DOUBLEHEX_MODULES}
  ${CRAYSIM_MATHS_ANTBODY_MODULES}
  ${CRAYSIM_ANTBODY_MODULES}
)
list(REMOVE_DUPLICATES ANTEYES_MODULES)

target_sources(ant_eyes PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR} FILES ${ANTEYES_MODULES})
target_link_libraries(ant_eyes ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} compound-ray::EyeRenderer3 glfw glad2 mplot_fontdata2)
