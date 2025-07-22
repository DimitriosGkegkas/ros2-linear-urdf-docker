cmake_minimum_required(VERSION 3.8)
project(linear_algebra_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(linear_algebra_interfaces REQUIRED)
add_executable(client src/client.cpp)
add_executable(server src/server.cpp)

target_link_libraries(server Eigen3::Eigen)
target_link_libraries(client Eigen3::Eigen)


ament_target_dependencies(client
  rclcpp
  linear_algebra_interfaces
  geometry_msgs
  Eigen3
)
ament_target_dependencies(server
  rclcpp
  linear_algebra_interfaces
  geometry_msgs
  Eigen3
)

install(TARGETS client server
  DESTINATION lib/${PROJECT_NAME})


ament_package()
