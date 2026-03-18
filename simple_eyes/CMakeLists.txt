include_directories (BEFORE "${PROJECT_SOURCE_DIR}/mathplot")

set(ANTEYES_VM_MODULES
  ${MPLOT_INCLUDE_PATH}/maths/sm/binomial
  ${MPLOT_INCLUDE_PATH}/maths/sm/nm_simplex
  ${MPLOT_INCLUDE_PATH}/maths/sm/bezcoord
  ${MPLOT_INCLUDE_PATH}/maths/sm/bezcurve
  ${MPLOT_INCLUDE_PATH}/maths/sm/bezcurvepath
  ${MPLOT_INCLUDE_PATH}/maths/sm/hex
  ${MPLOT_INCLUDE_PATH}/maths/sm/hexgrid
  ${MPLOT_INCLUDE_PATH}/maths/sm/hexgrid_hdf

  ${PROJECT_SOURCE_DIR}/modules/craysim_antbody.cppm
  ${PROJECT_SOURCE_DIR}/modules/craysim_doublehexgrid.cppm

  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/interop.h
  ${MPLOT_INCLUDE_PATH}/mplot/VerticesVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/ScatterVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/QuiverVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/HexGridVisual.h
  ${MPLOT_INCLUDE_PATH}/mplot/LengthscaleVisual.h
)

# Each C++ module file has to be listed here (in case they don't have a .cppm or .ixx file suffix)
set_source_files_properties (
  ${MY_MODULES}
  ${OCES_MODULES}
  ${MATHS_CORE_MODULES}
  ${MATHS_MATHPLOT_MODULES}
  ${MATHPLOT_CORE_MODULES}
  ${ANTEYES_VM_MODULES}
  PROPERTIES LANGUAGE CXX
)

# Build glad library here
include_directories(BEFORE ${MPLOT_INCLUDE_PATH}/mplot/glad)
add_library(glad2 STATIC ${MPLOT_INCLUDE_PATH}/mplot/glad/gl.c)

# Build mathplot fonts data
add_library(mplot_fontdata2 STATIC ${MPLOT_INCLUDE_PATH}/mplot/VisualFaceAsm.cpp)

add_executable (ant_eyes ant_eyes.cpp)
set(ANTEYES_MODULES ${MATHS_CORE_MODULES} ${MATHS_MATHPLOT_MODULES} ${MATHPLOT_CORE_MODULES} ${ANTEYES_VM_MODULES})
target_sources(ant_eyes PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR} FILES ${ANTEYES_MODULES})
target_link_libraries(ant_eyes ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} compound-ray::EyeRenderer3 glfw glad2 mplot_fontdata2)
