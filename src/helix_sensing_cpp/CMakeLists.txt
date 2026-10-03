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

# ROS-free detection core and Python-compatible number formatting/parsing.
add_library(helix_sensing_core STATIC
  src/anomaly_core.cpp
  src/pyfmt.cpp
)
target_include_directories(helix_sensing_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(helix_sensing_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Component library: AnomalyDetectorNode
add_library(anomaly_detector_component SHARED
  src/anomaly_detector_node.cpp
)
target_include_directories(anomaly_detector_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(anomaly_detector_component helix_sensing_core)
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

# Replay / benchmark tool for the core (no ROS): parity tests and benchmarks.
add_executable(helix_anomaly_replay tools/anomaly_replay.cpp)
target_link_libraries(helix_anomaly_replay helix_sensing_core)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS helix_anomaly_replay
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS anomaly_detector_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable copyright check (no LICENSE in package): other linters still run.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  # Pure math kernel
  ament_add_gtest(test_rolling_stats test/test_rolling_stats.cpp)
  target_include_directories(test_rolling_stats PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  # Python-compatible formatting and float() parsing
  ament_add_gtest(test_pyfmt test/test_pyfmt.cpp)
  target_link_libraries(test_pyfmt helix_sensing_core)

  # Native core vs the Python reference node's own methods, sample by sample.
  ament_add_pytest_test(test_python_parity test/test_python_parity.py
    ENV HELIX_ANOMALY_REPLAY=$<TARGET_FILE:helix_anomaly_replay>
    TIMEOUT 300
  )

  # Heartbeat emitter (CRASH-detection contract with HeartbeatMonitor)
  ament_add_gtest(test_heartbeat test/test_heartbeat.cpp)
  target_include_directories(test_heartbeat PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
  target_link_libraries(test_heartbeat anomaly_detector_component)
  # Own DDS domain: test_anomaly_detector_node activates nodes with the
  # same name, and ctest runs the binaries concurrently, so its beats
  # would otherwise be counted here.
  set_tests_properties(test_heartbeat PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=87"
  )
  ament_target_dependencies(test_heartbeat
    rclcpp
    rclcpp_lifecycle
    rclcpp_components
    std_msgs
    diagnostic_msgs
    lifecycle_msgs
    helix_msgs
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
  # Own DDS domain, see the note on test_heartbeat above.
  set_tests_properties(test_anomaly_detector_node PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=86"
  )
endif()

ament_package()
