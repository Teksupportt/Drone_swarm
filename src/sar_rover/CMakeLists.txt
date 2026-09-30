cmake_minimum_required(VERSION 3.8)
project(sar_rover)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sar_interfaces REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
add_executable(rover_node
  src/main.cpp
  src/rover_node.cpp
)

target_include_directories(rover_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rover_node
  rclcpp
  sar_interfaces
  nav2_msgs
  rclcpp_action
  geometry_msgs
)
install(TARGETS rover_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(exploration_node
  src/exploration_node.cpp
)
target_include_directories(exploration_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(exploration_node
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
)
install(TARGETS exploration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
ament_package()
