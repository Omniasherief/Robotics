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
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(pubc src/pubc.cpp)
ament_target_dependencies(pubc rclcpp std_msgs)

add_executable(subc src/subc.cpp)
ament_target_dependencies(subc rclcpp std_msgs)


add_executable(custom_msg src/custom_msg.cpp)
ament_target_dependencies(custom_msg rclcpp std_msgs lab3_custom_msg_pkg)

add_executable(service_server src/add_two_int_server.cpp)
ament_target_dependencies(service_server rclcpp std_msgs lab3_custom_msg_pkg)


add_executable(service_client src/add_two_int_client.cpp)
ament_target_dependencies(service_client rclcpp std_msgs lab3_custom_msg_pkg)

add_executable(action_server src/action_server.cpp)
ament_target_dependencies(action_server  rclcpp std_msgs lab3_custom_msg_pkg   rclcpp_action)

add_executable(action_client src/action_client.cpp)
ament_target_dependencies(action_client  rclcpp std_msgs lab3_custom_msg_pkg   rclcpp_action)

install(TARGETS
  pubc
  subc
  custom_msg
  service_server
  service_client
  action_server 
  action_client
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY 
launch
DESTINATION share/${PROJECT_NAME}/)

ament_package()
