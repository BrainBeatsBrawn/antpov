include_directories (BEFORE "${PROJECT_SOURCE_DIR}/mathplot")
add_executable (make_cyl_eye make_cyl_eye.cpp)
add_executable (make_poly_eye make_poly_eye.cpp)

include_directories (BEFORE "${PROJECT_SOURCE_DIR}/mplotext")
add_executable (make_fourpi_eye make_fourpi_eye.cpp)
add_executable (make_foveated_cutout make_foveated_cutout.cpp)
add_executable (make_zernike make_zernike.cpp)
target_link_libraries(make_zernike ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glfw)

add_executable (ant_eyes ant_eyes.cpp)
target_link_libraries(ant_eyes ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glfw)
