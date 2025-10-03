cmake_minimum_required(VERSION 3.8)
project(task1_rpm_speed)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(rpm_publisher src/rpm_publisher.cpp)
ament_target_dependencies(rpm_publisher  rclcpp std_msgs )

add_executable(speed_calculator src/speed_calculator.cpp)
ament_target_dependencies(speed_calculator  rclcpp std_msgs )




install(TARGETS
  rpm_publisher
  speed_calculator

  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY 
launch
DESTINATION share/${PROJECT_NAME}/)

ament_package()
