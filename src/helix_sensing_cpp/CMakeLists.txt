cmake_minimum_required(VERSION 3.8)
project(helix_sensing_cpp)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(helix_msgs REQUIRED)

# Component library: AnomalyDetectorNode
add_library(anomaly_detector_component SHARED
  src/anomaly_detector_node.cpp
)
target_include_directories(anomaly_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(anomaly_detector_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  diagnostic_msgs
  lifecycle_msgs
  helix_msgs
)
rclcpp_components_register_node(anomaly_detector_component
  PLUGIN "helix_sensing_cpp::AnomalyDetectorNode"
  EXECUTABLE helix_anomaly_detector
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS anomaly_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable copyright check (no LICENSE in package) — other linters still run.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Pure math kernel
  ament_add_gtest(test_rolling_stats test/test_rolling_stats.cpp)
  target_include_directories(test_rolling_stats PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  # Node integration
  ament_add_gtest(test_anomaly_detector_node test/test_anomaly_detector_node.cpp)
  target_include_directories(test_anomaly_detector_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_anomaly_detector_node anomaly_detector_component)
  ament_target_dependencies(test_anomaly_detector_node
    rclcpp
    rclcpp_lifecycle
    rclcpp_components
    std_msgs
    diagnostic_msgs
    lifecycle_msgs
    helix_msgs
  )
endif()

ament_package()
