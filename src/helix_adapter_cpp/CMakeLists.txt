cmake_minimum_required(VERSION 3.8)
project(helix_adapter_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

add_library(rate_window src/rate_window.cpp)
target_include_directories(rate_window PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(helix_topic_rate_monitor src/topic_rate_monitor_node.cpp)
target_include_directories(helix_topic_rate_monitor PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(helix_topic_rate_monitor rate_window)
ament_target_dependencies(helix_topic_rate_monitor
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  lifecycle_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS rate_window
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS helix_topic_rate_monitor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_rate_window test/test_rate_window.cpp)
  target_include_directories(test_rate_window PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_rate_window rate_window)
endif()

ament_package()
