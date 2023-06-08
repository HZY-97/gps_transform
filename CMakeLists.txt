cmake_minimum_required(VERSION 3.0.2)
project(gps_transform)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

set(OPEN_LOG 1) # set 1 open log ; set 0 close log

if(OPEN_LOG)
    add_compile_definitions(OPEN_LOG)
endif()

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
)
set(EIGEN_INCLUDE_DIRS /usr/include/eigen3)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
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
)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SOURCE)
# message("===================${SOURCE}============================")

add_executable(${PROJECT_NAME}_node ${SOURCE})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})