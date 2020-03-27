rock_add_ruby_package(rock_gazebo)
if (rock_gazebo_AVAILABLE)
    rock_ruby_library(rock)
endif()

configure_file(path_to_plugin.rb.in path_to_plugin.rb @ONLY)
if (rock_gazebo_AVAILABLE)
    install(PROGRAMS bin/rock-gazebo-viz bin/rock-gazebo-run bin/rock-gazebo
        bin/rock-gzserver
        DESTINATION bin
        PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
                    GROUP_READ GROUP_EXECUTE
                    WORLD_READ WORLD_EXECUTE)
    install(FILES ${CMAKE_CURRENT_BINARY_DIR}/path_to_plugin.rb
        DESTINATION ${RUBY_LIBRARY_INSTALL_DIR}/rock_gazebo)
endif()

