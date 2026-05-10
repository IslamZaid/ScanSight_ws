cmake_minimum_required(VERSION 3.0.2)
project(laser_event_processing)

find_package(catkin REQUIRED COMPONENTS roscpp dv_ros_msgs dv_ros_messaging)
find_package(dv-processing 2.0.0 REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp dv_ros_msgs dv_ros_messaging)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(event_crop_node src/event_crop_node.cpp)
target_link_libraries(event_crop_node ${catkin_LIBRARIES})

add_executable(event_filter_node src/event_filter_node.cpp)
target_link_libraries(event_filter_node ${catkin_LIBRARIES} dv::processing)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
