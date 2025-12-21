cmake_minimum_required(VERSION 3.8)
project(my_robot_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

# 2. Include Directories
include_directories(include)

# 3. Define Executable and Dependencies
add_executable(pd_motion_planner src/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner 
  rclcpp 
  nav_msgs 
  geometry_msgs 
  tf2 
  tf2_ros
)

# 4. Install Commands
install(
  DIRECTORY include
  DESTINATION include
)

install(TARGETS
  pd_motion_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()