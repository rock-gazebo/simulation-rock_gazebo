cmake_minimum_required(VERSION 3.28)
project(rock_gazebo
        VERSION 0.3
	DESCRIPTION "Gz plugin that creates Rock components to interface with Gz simulations")
find_package(Rock)
rock_init()
rock_standard_layout()
