add_definitions(-DRTT_COMPONENT)
rock_library(gz_rock
	SOURCES RockSystem.cpp details.cpp
    HEADERS RockSystem.hpp
	DEPS_PKGCONFIG
                orocos-rtt-gnulinux
                orocos-rtt-corba-gnulinux
                std-typekit-gnulinux
                std-transport-corba-gnulinux
                std-transport-mqueue-gnulinux
                std-transport-typelib-gnulinux
                base-typekit-gnulinux
                base-transport-corba-gnulinux
                base-transport-mqueue-gnulinux
                base-transport-typelib-gnulinux
                gps_base-typekit-gnulinux
                gps_base-transport-corba-gnulinux
                gps_base-transport-mqueue-gnulinux
                gps_base-transport-typelib-gnulinux
                gz_rock-tasks-gnulinux
                gz_rock-typekit-gnulinux
                gz_rock-transport-corba-gnulinux
                gz_rock-transport-mqueue-gnulinux
                gz_rock-transport-typelib-gnulinux
                logger-tasks-gnulinux
                logger-typekit-gnulinux
                logger-transport-corba-gnulinux
                logger-transport-mqueue-gnulinux
                logger-transport-typelib-gnulinux
	DEPS_CMAKE Boost gz-sim)

rock_library(gz_rock-plugintask
	SOURCES PluginTask.cpp details.cpp
    HEADERS PluginTask.hpp
	DEPS_PKGCONFIG
                orocos-rtt-gnulinux
                orocos-rtt-corba-gnulinux
	DEPS_CMAKE gz-sim)
