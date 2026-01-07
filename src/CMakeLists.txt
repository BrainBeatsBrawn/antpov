include_directories (BEFORE "${PROJECT_SOURCE_DIR}/extern/oces_viewer")
include_directories (BEFORE "${PROJECT_SOURCE_DIR}/extern/oces_viewer/extern")

# The OptiX way to create an executable
OPTIX_add_sample_executable (antpov target_name antpov.cpp OPTIONS -rdc true)

# Link CUDA, mathplot dependencies and libEyeRenderer3.so omit: ${CUDA_LIBRARIES}
target_link_libraries(${target_name} ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} compound-ray::EyeRenderer3)

add_executable (fiducials fiducials.cpp)
target_link_libraries(fiducials ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glfw)
