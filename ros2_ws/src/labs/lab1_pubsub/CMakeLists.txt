cmake_minimum_required(VERSION 3.8)
project(lab1_pubsub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lab3_custom_msg_pkg REQUIRED)

add_executable(pubc src/pubc.cpp)
ament_target_dependencies(pubc rclcpp std_msgs)

add_executable(subc src/subc.cpp)
ament_target_dependencies(subc rclcpp std_msgs)


add_executable(custom_msg src/custom_msg.cpp)
ament_target_dependencies(custom_msg rclcpp std_msgs lab3_custom_msg_pkg)



install(TARGETS
  pubc
  subc
  custom_msg
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
