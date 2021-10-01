cmake_minimum_required(VERSION 3.5.1)
project(rotate_pcd)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

catkin_package()

add_executable(rotate_pcd src/main.cpp)

target_include_directories(rotate_pcd
  PRIVATE ${PCL_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(rotate_pcd
  PRIVATE ${PCL_LIBRARY_DIRS}
  PRIVATE Eigen3::Eigen
)

## Install ##
install(TARGETS rotate_pcd
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
