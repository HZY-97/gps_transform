cmake_minimum_required(VERSION 3.0.2)
project(gps_transform)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")


find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
)

find_package(PCL REQUIRED)

set(EIGEN_INCLUDE_DIRS /usr/include/eigen3)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sino
#  CATKIN_DEPENDS nav_msgs roscpp rospy
#  DEPENDS system_lib
)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${EIGEN_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}/pcl-1.8
  )

link_directories(${PCL_LIBRARY_DIRS})
  
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src/gpsTransform SOURCE)
# message("===================${SOURCE}============================")

add_executable(${PROJECT_NAME}_node ${SOURCE})
target_link_libraries(${PROJECT_NAME}_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/Tool)