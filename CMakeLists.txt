cmake_minimum_required(VERSION 3.8)
project(auv_vision_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(lane_planner src/lane_planner.cpp)
target_compile_features(lane_planner PUBLIC cxx_std_17)
target_include_directories(lane_planner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(lane_vision_controller_node src/lane_vision_controller_node.cpp)
target_compile_features(lane_vision_controller_node PUBLIC cxx_std_17)
target_include_directories(lane_vision_controller_node
  PRIVATE include
)
target_link_libraries(lane_vision_controller_node lane_planner)
ament_target_dependencies(
  lane_vision_controller_node
  geometry_msgs
  mavros_msgs
  nav_msgs
  rclcpp
  std_msgs
)

install(
  TARGETS lane_planner lane_vision_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(PROGRAMS
  scripts/yolo_buoy_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_buoy_detector
)

install(PROGRAMS
  scripts/ros_image_webrtc_streamer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ros_image_webrtc_streamer
)

install(FILES
  scripts/yolo_webrtc_transport.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_yolo_env.py
  scripts/setup_laptop_yolo_env.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  requirements-yolo.txt
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
