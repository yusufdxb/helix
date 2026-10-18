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
find_package(rcl_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

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

# Component library: TopicRateMonitorNode (generic subscriptions, no deserialization)
add_library(topic_rate_monitor_component SHARED
  src/topic_rate_monitor_node.cpp
)
target_include_directories(topic_rate_monitor_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(topic_rate_monitor_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  helix_msgs
)
rclcpp_components_register_node(topic_rate_monitor_component
  PLUGIN "helix_sensing_cpp::TopicRateMonitorNode"
  EXECUTABLE helix_topic_rate_monitor
)

# Component library: LogParserNode (/rosout -> LOG_PATTERN faults)
add_library(log_matcher STATIC src/log_matcher.cpp)
target_include_directories(log_matcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(log_matcher yaml-cpp)
set_target_properties(log_matcher PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_library(log_parser_component SHARED src/log_parser_node.cpp)
target_link_libraries(log_parser_component log_matcher)
ament_target_dependencies(log_parser_component
  rclcpp rclcpp_lifecycle rclcpp_components std_msgs rcl_interfaces helix_msgs)
rclcpp_components_register_node(log_parser_component
  PLUGIN "helix_sensing_cpp::LogParserNode"
  EXECUTABLE helix_log_parser
)
add_executable(helix_log_replay tools/log_replay.cpp)
target_link_libraries(helix_log_replay log_matcher)

# Replay / benchmark tool for the core (no ROS): parity tests and benchmarks.
add_executable(helix_anomaly_replay tools/anomaly_replay.cpp)
target_link_libraries(helix_anomaly_replay helix_sensing_core)

install(DIRECTORY include/
  DESTINATION include
)

# Offline detector evaluation over recorded streams (analysis/detector_eval.py).
add_executable(helix_detector_eval tools/detector_eval.cpp)
target_link_libraries(helix_detector_eval helix_sensing_core)

install(TARGETS helix_anomaly_replay helix_detector_eval helix_log_replay
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS anomaly_detector_component topic_rate_monitor_component log_parser_component
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

  # Detector extensions (freeze, std floor, clear records, anchored band)
  ament_add_gtest(test_anomaly_core test/test_anomaly_core.cpp)
  target_link_libraries(test_anomaly_core helix_sensing_core)

  # Native log parser vs helix_core.log_parser, record by record
  ament_add_pytest_test(test_log_parser_parity test/test_log_parser_parity.py
    ENV HELIX_LOG_REPLAY=$<TARGET_FILE:helix_log_replay>
    TIMEOUT 120
  )

  # Rate window contract (same as helix_adapter.rate_window)
  ament_add_gtest(test_rate_window test/test_rate_window.cpp)
  target_include_directories(test_rate_window PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
  )

  ament_add_pytest_test(test_rate_monitor_teardown test/test_rate_monitor_teardown.py
    ENV RATE_MONITOR_EXECUTABLE=$<TARGET_FILE:helix_topic_rate_monitor>
      ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    TIMEOUT 180
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
