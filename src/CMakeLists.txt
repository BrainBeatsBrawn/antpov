# Build glad library here
include_directories(BEFORE ${MPLOT_INCLUDE_PATH}/mplot/glad)
add_library(glad STATIC ${MPLOT_INCLUDE_PATH}/mplot/glad/gl.c)

# Build mathplot fonts data
add_library(mplot_fontdata STATIC ${MPLOT_INCLUDE_PATH}/mplot/VisualFaceAsm.cpp)

# The main ant simulator
set(ANTPOV_MODULES
  ${MPLOT_MATHS_CORE_MODULES}
  ${MPLOT_MATHS_VISUALMODEL_MODULES}
  ${MPLOT_CORE_MODULES}
  ${CRAYSIM_MATHS_MODULES}
  ${CRAYSIM_MATHPLOT_MODULES}
  ${CRAYSIM_MODULES}
  ${CRAYSIM_MATHS_ANTBODY_MODULES}
  ${CRAYSIM_ANTBODY_MODULES}
  ${OCES_MODULES}
  ${ANTPOV_VM_MODULES}
  ${MY_MODULES}
)
list(REMOVE_DUPLICATES ANTPOV_MODULES)

add_library(antpov_modules STATIC)
target_sources(antpov_modules
  PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES ${ANTPOV_MODULES}
)
target_link_libraries(antpov_modules ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} compound-ray::EyeRenderer3 glad mplot_fontdata)

add_executable(antpov antpov.cpp)
target_link_libraries(antpov antpov_modules)

# A special exe to make fiducials for the Seville environment
add_executable(fiducials fiducials.cpp)
target_link_libraries(fiducials antpov_modules)

# Ant direction processing test
set(DIRNS_MODULES
  ${MPLOT_MATHS_CORE_MODULES}
  ${MPLOT_CORE_MODULES}
  ${MPLOT_MATHS_VISUALMODEL_MODULES}
  ${MPLOT_INCLUDE_PATH}/mplot/GraphVisual.cppm
  ${MPLOT_INCLUDE_PATH}/mplot/GridVisual.cppm
  ${MY_MODULES}
)
list(REMOVE_DUPLICATES DIRNS_MODULES)
add_library(dirns_modules STATIC)
target_sources(dirns_modules
  PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES ${DIRNS_MODULES}
)
target_link_libraries(dirns_modules ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glad mplot_fontdata)

add_executable(dirns dirns.cpp)
target_link_libraries(dirns dirns_modules)

add_executable(cater_paths cater_paths.cpp)
target_link_libraries(cater_paths dirns_modules)

add_executable(cater_sel_paths cater_sel_paths.cpp)
target_link_libraries(cater_sel_paths dirns_modules)

add_executable(cater_graph cater_graph.cpp)
target_link_libraries(cater_graph dirns_modules)
