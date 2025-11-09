cmake_minimum_required(VERSION 3.8)
project(lab8_my_robot_mapping)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(map_create src/occupancy_grid_pub.cpp)
ament_target_dependencies(map_create rclcpp nav_msgs std_msgs)

install(
  DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  map_create
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
