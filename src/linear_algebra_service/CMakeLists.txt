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
add_executable(client src/client.cpp)
add_executable(server src/server.cpp)

ament_target_dependencies(client
  rclcpp
)

install(TARGETS client server
  DESTINATION lib/${PROJECT_NAME})


ament_package()
