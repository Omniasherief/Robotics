cmake_minimum_required(VERSION 3.8)
project(task2_camera_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)


# Generate service
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CameraAngle.srv"
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# executables
add_executable(camera_client src/camera_service_client.cpp)
ament_target_dependencies(camera_client rclcpp std_msgs sensor_msgs cv_bridge)

target_link_libraries(camera_client
  ${OpenCV_LIBRARIES}
)

add_executable(camera_server src/camera_service_server.cpp)
ament_target_dependencies(camera_server rclcpp std_msgs sensor_msgs cv_bridge)

target_link_libraries(camera_server
  ${OpenCV_LIBRARIES}
)

## Ensure executables depend on generated interfaces
rosidl_target_interfaces(camera_client
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(camera_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# install
install(TARGETS
  camera_client
  camera_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()