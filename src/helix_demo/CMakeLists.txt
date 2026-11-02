cmake_minimum_required(VERSION 3.8)
project(helix_demo CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(demo_node src/demo_node.cpp)
ament_target_dependencies(demo_node rclcpp rclcpp_lifecycle std_msgs)
install(TARGETS demo_node DESTINATION lib/${PROJECT_NAME})

ament_package()
