cmake_minimum_required(VERSION 3.5.1)
project(transform_pcd)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS pcl_ros)
find_package(PCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem program_options)

catkin_package(
  CATKIN_DEPENDS pcl_ros
)

add_executable(transform_pcd src/main.cpp)

target_include_directories(transform_pcd
  PRIVATE ${catkin_INCLUDE_DIRS}
  PRIVATE ${PCL_INCLUDE_DIRS}
  PRIVATE ${Boost_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(transform_pcd
  PRIVATE ${catkin_LIBRARIES}
  PRIVATE ${PCL_LIBRARY_DIRS}
  PRIVATE Eigen3::Eigen
  PRIVATE ${Boost_LIBRARIES}
)

## Install ##
install(TARGETS transform_pcd
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
