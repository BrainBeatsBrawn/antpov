include_directories (BEFORE "${PROJECT_SOURCE_DIR}/mplotext")
# The OptiX way to create an executable
OPTIX_add_sample_executable (eye3d target_name eye3d.cpp OPTIONS -rdc true)

# Link CUDA, mathplot dependencies and libEyeRenderer3.so omit: ${CUDA_LIBRARIES}
target_link_libraries(${target_name} ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} compound-ray::EyeRenderer3)
