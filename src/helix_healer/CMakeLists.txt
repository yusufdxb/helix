cmake_minimum_required(VERSION 3.8)
project(helix_healer CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(helix_msgs REQUIRED)

# ROS-free core: spec parser, diagnosis/repair policy, process supervision.
add_library(helix_healer_core STATIC src/spec.cpp src/policy.cpp src/process.cpp src/envelope.cpp)
target_include_directories(helix_healer_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(helix_healer_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(helix_healer_node src/healer_node.cpp)
target_link_libraries(helix_healer_node helix_healer_core)
ament_target_dependencies(helix_healer_node rclcpp ament_index_cpp lifecycle_msgs diagnostic_msgs std_msgs helix_msgs)

install(TARGETS helix_healer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_healer test/test_spec.cpp test/test_policy.cpp test/test_process.cpp test/test_envelope.cpp)
  target_link_libraries(test_healer helix_healer_core)
endif()

ament_package()
