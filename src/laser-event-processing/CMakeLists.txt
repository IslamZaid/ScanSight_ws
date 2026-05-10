cmake_minimum_required(VERSION 3.0.2)
project(laser_event_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  dv_ros_msgs
  dv_ros_messaging
  geometry_msgs
  std_msgs
  message_generation
)
find_package(dv-processing 2.0.0 REQUIRED)

add_message_files(FILES
  EventV.msg
  EventVArray.msg
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS roscpp dv_ros_msgs dv_ros_messaging geometry_msgs std_msgs message_runtime
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(event_crop_node src/event_crop_node.cpp)
target_link_libraries(event_crop_node ${catkin_LIBRARIES})
add_dependencies(event_crop_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(event_filter_node src/event_filter_node.cpp)
target_link_libraries(event_filter_node ${catkin_LIBRARIES} dv::processing)
add_dependencies(event_filter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(event_velocity_node src/event_velocity_node.cpp)
target_link_libraries(event_velocity_node ${catkin_LIBRARIES})
add_dependencies(event_velocity_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(DIRECTORY launch msg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
