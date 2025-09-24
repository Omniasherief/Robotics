cmake_minimum_required(VERSION 3.8)
project(lab2_turtlesim_pub_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtle_mover src/turtle_pub.cpp)
ament_target_dependencies(turtle_mover rclcpp geometry_msgs)

add_executable(param src/param.cpp)
ament_target_dependencies(param rclcpp geometry_msgs)

install(TARGETS
  turtle_mover
  param
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
