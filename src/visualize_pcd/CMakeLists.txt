cmake_minimum_required(VERSION 3.0.2)
project(visualize_pcd)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros
)

## Build ##

add_executable(visualize_pcd src/main.cpp)

target_include_directories(visualize_pcd
  PRIVATE ${catkin_INCLUDE_DIRS}
  PRIVATE ${PCL_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(visualize_pcd
  PRIVATE ${catkin_LIBRARIES}
  PRIVATE ${PCL_LIBRARY_DIRS}
)

## Install ##
install(TARGETS visualize_pcd
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
