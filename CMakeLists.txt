# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 3.9)
project(rock_gazebo
        VERSION 0.2
        DESCRIPTION "C++ gazebo plugin that interfaces Gazebo with a Rock system")
find_package(Rock)
rock_init()

option(BUILD_GAZEBO_PLUGIN "build the C++ gazebo plugin" ON)
if (BUILD_GAZEBO_PLUGIN)
    rock_standard_layout()
else()
    option(BINDINGS_RUBY "install this package's Ruby bindings" ON)
    include(RockRuby)
    if (BINDINGS_RUBY)
        if (RUBY_FOUND)
            add_subdirectory(bindings/ruby)
        else()
            message(FATAL_ERROR "this package has Ruby bindings but Ruby cannot be found. Set BINDINGS_RUBY to OFF (e.g.  -DBINDINGS_RUBY=OFF to avoid installing the bindings")
        endif()
    endif()
endif()
