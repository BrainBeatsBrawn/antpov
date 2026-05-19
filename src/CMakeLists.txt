# Each C++ module file has to be listed here (as they don't have a .cppm or .ixx file suffix)
set_source_files_properties (
  ${MPLOT_MATHS_CORE_MODULES}
  ${MPLOT_MATHS_VISUALMODEL_MODULES}
  ${MPLOT_CORE_MODULES}
  ${OCES_MODULES}
  ${ANTPOV_VM_MODULES}
  ${FIDUCIALS_VM_MODULES}
  ${MY_MODULES}
  PROPERTIES LANGUAGE CXX
)

# Build glad library here
include_directories(BEFORE ${MPLOT_INCLUDE_PATH}/mplot/glad)
add_library(glad STATIC ${MPLOT_INCLUDE_PATH}/mplot/glad/gl.c)

# Build mathplot fonts data
add_library(mplot_fontdata STATIC ${MPLOT_INCLUDE_PATH}/mplot/VisualFaceAsm.cpp)

# A special exe to make fiducials for the Seville environment
add_executable(fiducials fiducials.cpp)
set(FIDUCIALS_MODULES ${MPLOT_MATHS_CORE_MODULES} ${MPLOT_MATHS_VISUALMODEL_MODULES} ${MPLOT_CORE_MODULES} ${FIDUCIALS_VM_MODULES})
list(REMOVE_DUPLICATES FIDUCIALS_MODULES)
target_sources(fiducials PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR} FILES ${FIDUCIALS_MODULES})
target_link_libraries(fiducials ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glfw glad mplot_fontdata)

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
add_executable(antpov antpov.cpp)
target_sources(antpov
  PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES ${ANTPOV_MODULES}
)
target_link_libraries(antpov ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} compound-ray::EyeRenderer3 glad mplot_fontdata)

# Ant direction processing test
set(DIRNS_MODULES
  ${MPLOT_MATHS_CORE_MODULES}
  ${MPLOT_CORE_MODULES}
  ${MPLOT_MATHS_VISUALMODEL_MODULES}
  ${MPLOT_INCLUDE_PATH}/mplot/GraphVisual.cppm
  ${MY_MODULES}
)
list(REMOVE_DUPLICATES DIRNS_MODULES)
add_executable(dirns dirns.cpp)
target_sources(dirns
  PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES ${DIRNS_MODULES}
)
target_link_libraries(dirns ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glad mplot_fontdata)

add_executable(cater_paths cater_paths.cpp)
target_sources(cater_paths
  PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES ${DIRNS_MODULES}
)
target_link_libraries(cater_paths ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glad mplot_fontdata)

add_executable(cater_sel_paths cater_sel_paths.cpp)
target_sources(cater_sel_paths
  PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES ${DIRNS_MODULES} ${MPLOT_INCLUDE_PATH}/mplot/GridVisual.cppm

)
target_link_libraries(cater_sel_paths ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glad mplot_fontdata)

add_executable(cater_graph cater_graph.cpp)
target_sources(cater_graph
  PUBLIC FILE_SET CXX_MODULES BASE_DIRS ${PROJECT_SOURCE_DIR}
  FILES ${DIRNS_MODULES}
)
target_link_libraries(cater_graph ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glad mplot_fontdata)
