cmake_minimum_required(VERSION 3.0.2)
project(event_crop)

find_package(catkin REQUIRED COMPONENTS roscpp dv_ros_msgs)
catkin_package(CATKIN_DEPENDS roscpp dv_ros_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(event_crop_node src/event_crop_node.cpp)
target_link_libraries(event_crop_node ${catkin_LIBRARIES})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
