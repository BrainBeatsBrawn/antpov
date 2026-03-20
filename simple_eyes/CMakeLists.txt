include_directories (BEFORE "${PROJECT_SOURCE_DIR}/mathplot")

set(ANTEYES_VM_MODULES
  ${MATHS_MODULE_PATH}/sm/binomial.cppm
  ${MATHS_MODULE_PATH}/sm/nm_simplex.cppm
  ${MATHS_MODULE_PATH}/sm/bezcoord.cppm
  ${MATHS_MODULE_PATH}/sm/bezcurve.cppm
  ${MATHS_MODULE_PATH}/sm/bezcurvepath.cppm
  ${MATHS_MODULE_PATH}/sm/hex.cppm
  ${MATHS_MODULE_PATH}/sm/hexgrid.cppm
  ${MATHS_MODULE_PATH}/sm/hexgrid_hdf.cppm

  ${CRAYSIM_MODULE_PATH}/modules/craysim_antbody.cppm
  ${CRAYSIM_MODULE_PATH}/modules/craysim_doublehexgrid.cppm

  ${MPLOT_INCLUDE_PATH}/mplot/compoundray/interop.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/VerticesVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/ScatterVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/QuiverVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/HexGridVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/LengthscaleVisual.cppm
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
