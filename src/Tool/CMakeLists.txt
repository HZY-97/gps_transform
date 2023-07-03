add_executable(icpGpsLidarPath_Tool ${CMAKE_CURRENT_SOURCE_DIR}/icpGpsLidarPath.cpp)
target_link_libraries(icpGpsLidarPath_Tool ${PCL_LIBRARIES} ${catkin_LIBRARIES})