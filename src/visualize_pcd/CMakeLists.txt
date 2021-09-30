cmake_minimum_required(VERSION 3.0.2)
project(visualize_pcd)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES visualize_pcd
  CATKIN_DEPENDS roscpp pcl_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_executable(visualize_pcd src/main.cpp)

## Specify libraries to link a library or executable target against
#target_link_libraries(visualize_pcd
#   ${catkin_LIBRARIES}
#)
