cmake_minimum_required(VERSION 3.8)
project(helix_monitor CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)

# ROS-free core: checks, stop verifier, config. Everything that decides lives here.
add_library(helix_monitor_core STATIC src/checks.cpp src/stop_verifier.cpp src/config.cpp)
target_include_directories(helix_monitor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(helix_monitor_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(monitor_eval tools/monitor_eval.cpp)
target_link_libraries(monitor_eval helix_monitor_core)

install(TARGETS monitor_eval DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME} OPTIONAL)

# Thin ROS node: subscribes to the GO2 topics, feeds the core, publishes FaultEvent and stop verdicts.
# It needs the robot's message packages; without them only the core and the eval tool are built.
find_package(rclcpp QUIET)
find_package(ament_index_cpp QUIET)
find_package(sensor_msgs QUIET)
find_package(helix_msgs QUIET)
find_package(unitree_go QUIET)
find_package(unitree_api QUIET)
if(rclcpp_FOUND AND ament_index_cpp_FOUND AND sensor_msgs_FOUND AND helix_msgs_FOUND AND unitree_go_FOUND AND unitree_api_FOUND)
  add_executable(helix_monitor_node src/monitor_node.cpp)
  target_link_libraries(helix_monitor_node helix_monitor_core)
  ament_target_dependencies(helix_monitor_node rclcpp ament_index_cpp sensor_msgs helix_msgs unitree_go unitree_api)
  install(TARGETS helix_monitor_node DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "rclcpp/sensor_msgs/helix_msgs/unitree_go/unitree_api not all found: helix_monitor_node is not built")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_monitor test/test_monitor.cpp)
  target_link_libraries(test_monitor helix_monitor_core)
endif()

ament_package()
