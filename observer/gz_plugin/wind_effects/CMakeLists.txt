
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

#============================================================================
# Initialize the project
#============================================================================
project(gz-sim7 VERSION 7.9.0)
set (GZ_DISTRIBUTION "Garden")

#============================================================================
# Find gz-cmake
#============================================================================
# If you get an error at this line, you need to install gz-cmake
find_package(gz-cmake3 REQUIRED)

#============================================================================
# Configure the project
#============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

gz_configure_project(VERSION_SUFFIX)


gz_find_package(sdformat13 REQUIRED)
set(SDF_VER ${sdformat13_VERSION_MAJOR})

#--------------------------------------
# Find gz-plugin
gz_find_package(gz-plugin2 REQUIRED COMPONENTS loader register)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

#--------------------------------------
# Find gz-transport
gz_find_package(gz-transport12 VERSION 12.1 REQUIRED COMPONENTS log parameters)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

#--------------------------------------
# Find gz-msgs
gz_find_package(gz-msgs9 REQUIRED)
set(GZ_MSGS_VER ${gz-msgs9_VERSION_MAJOR})

#--------------------------------------
# Find gz-common
# Always use the profiler component to get the headers, regardless of status.
gz_find_package(gz-common5
  COMPONENTS
    profiler
    events
    av
    io
  REQUIRED
)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

#--------------------------------------
# Find gz-fuel_tools
gz_find_package(gz-fuel_tools8 REQUIRED)
set(GZ_FUEL_TOOLS_VER ${gz-fuel_tools8_VERSION_MAJOR})

#--------------------------------------
# Find gz-gui
gz_find_package(gz-gui7 REQUIRED)
set(GZ_GUI_VER ${gz-gui7_VERSION_MAJOR})
gz_find_package (Qt5
  COMPONENTS
    Core
    Quick
    QuickControls2
  REQUIRED
  PKGCONFIG "Qt5Core Qt5Quick Qt5QuickControls2")

#--------------------------------------
# Find gz-physics
gz_find_package(gz-physics6
  COMPONENTS
    heightmap
    mesh
    sdf
  REQUIRED
)
set(GZ_PHYSICS_VER ${gz-physics6_VERSION_MAJOR})

#--------------------------------------
# Find gz-sensors
gz_find_package(gz-sensors7 REQUIRED VERSION 7.1
  # component order is important
  COMPONENTS
    # non-rendering
    air_pressure
    air_speed
    altimeter
    imu
    force_torque
    logical_camera
    magnetometer
    navsat

    # rendering
    rendering
    lidar
    gpu_lidar

    # cameras
    camera
    boundingbox_camera
    segmentation_camera
    depth_camera
    rgbd_camera
    thermal_camera
    wide_angle_camera
)
set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})

#--------------------------------------
# Find gz-rendering
gz_find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})

#--------------------------------------
# Find gz-math
gz_find_package(gz-math7 REQUIRED COMPONENTS eigen3)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

#--------------------------------------
# Find if gz command is available
find_program(GZ_TOOLS_PROGRAM gz)
if (GZ_TOOLS_PROGRAM)
  message (STATUS "Searching for gz program - found. CLI tests can be built.")
else()
  message (STATUS "Searching for gz program - not found. CLI tests are skipped.")
endif()
# Note that CLI files are installed regardless of whether the dependency is
# available during build time
set(GZ_TOOLS_VER 2)

#--------------------------------------
# Find gz-utils
gz_find_package(gz-utils2 REQUIRED COMPONENTS cli)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})

#--------------------------------------
# Find protobuf
gz_find_package(GzProtobuf
                REQUIRED
                COMPONENTS all
                PRETTY Protobuf)
set(Protobuf_IMPORT_DIRS ${gz-msgs9_INCLUDE_DIRS})

#============================================================================
# Create package information
#============================================================================
gz_create_packages()


gz_add_system(wind-effects
  SOURCES
    WindEffects.cc
  PUBLIC_LINK_LIBS
    gz-transport${GZ_TRANSPORT_VER}::gz-transport${GZ_TRANSPORT_VER}
    # Include gz-sensors for noise models
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
)

# gz_find_package(gz-plugin2 REQUIRED COMPONENTS register)
# set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})
 
# Add sources for each plugin to be registered.
# add_library(WindEffects WindEffects.cc)
# set_property(TARGET WindEffects PROPERTY CXX_STANDARD 17)
# target_link_libraries(WindEffects
#   gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
#   gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
# )

# configure_file(
#   ${CMAKE_CURRENT_BINARY_DIR}/include/gz/${GZ_DESIGNATION}.hh
#   ${CMAKE_CURRENT_BINARY_DIR}/include/gz/sim.hh
#   COPYONLY)
# install(
#   FILES ${CMAKE_CURRENT_BINARY_DIR}/include/gz/sim.hh
#   DESTINATION ${GZ_INCLUDE_INSTALL_DIR_FULL}/gz)
