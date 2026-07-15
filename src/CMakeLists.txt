# The main simulator program, antpov
add_executable(antpov antpov.cpp)
target_link_libraries(antpov antpov_modules)

add_executable(antpov_plotcoords antpov_plotcoords.cpp)
target_link_libraries(antpov_plotcoords antpov_modules)

# A special exe to make fiducials for the Seville environment
add_executable(fiducials fiducials.cpp)
target_link_libraries(fiducials antpov_modules)

add_executable(dirns dirns.cpp)
target_link_libraries(dirns antpov_modules)

add_executable(cater_paths cater_paths.cpp)
target_link_libraries(cater_paths antpov_modules)

add_executable(cater_sel_paths cater_sel_paths.cpp)
target_link_libraries(cater_sel_paths antpov_modules)

add_executable(cater_graph cater_graph.cpp)
target_link_libraries(cater_graph antpov_modules)
