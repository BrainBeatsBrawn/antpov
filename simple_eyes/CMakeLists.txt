include_directories (BEFORE "${PROJECT_SOURCE_DIR}/mathplot")

add_executable (ant_eyes ant_eyes.cpp)
target_link_libraries(ant_eyes ${MPLOT_LIBS_CORE} ${MPLOT_LIBS_GL} glfw)
