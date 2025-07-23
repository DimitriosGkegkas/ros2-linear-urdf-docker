cmake_minimum_required(VERSION 3.8)
project(ur20_display)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(joint_state_publisher src/joint_state_publisher.cpp)

ament_target_dependencies(joint_state_publisher
  rclcpp
  sensor_msgs
)

install(TARGETS joint_state_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
