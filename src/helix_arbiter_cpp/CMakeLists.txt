cmake_minimum_required(VERSION 3.8)
project(helix_arbiter_cpp)

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
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(helix_msgs REQUIRED)

# ROS-free core: every arbitration policy and the parameter contract.
add_library(helix_arbiter_core STATIC
  src/arbiter_core.cpp
  src/arbiter_config.cpp
)
target_include_directories(helix_arbiter_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(helix_arbiter_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

# rclcpp_lifecycle wrapper.
add_library(helix_arbiter_node STATIC src/arbiter_node.cpp)
target_link_libraries(helix_arbiter_node helix_arbiter_core)
ament_target_dependencies(helix_arbiter_node
  rclcpp
  rclcpp_lifecycle
  lifecycle_msgs
  rcl_interfaces
  geometry_msgs
  helix_msgs
)
set_target_properties(helix_arbiter_node PROPERTIES POSITION_INDEPENDENT_CODE ON)

# The node executable: ros2 run helix_arbiter_cpp helix_arbiter
add_executable(helix_arbiter src/arbiter_main.cpp)
target_link_libraries(helix_arbiter helix_arbiter_node)
ament_target_dependencies(helix_arbiter rclcpp lifecycle_msgs)

# Replay / benchmark tool for the core (no ROS): parity tests and benchmarks.
add_executable(helix_arbiter_replay tools/arbiter_replay.cpp)
target_link_libraries(helix_arbiter_replay helix_arbiter_core)

install(TARGETS helix_arbiter helix_arbiter_replay
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Disable copyright check (no LICENSE in package): other linters still run.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_arbiter_core test/test_arbiter_core.cpp)
  target_link_libraries(test_arbiter_core helix_arbiter_core)

  ament_add_gtest(test_arbiter_config test/test_arbiter_config.cpp)
  target_link_libraries(test_arbiter_config helix_arbiter_core)

  # Native core vs the Python reference, decision by decision.
  ament_add_pytest_test(test_python_parity test/test_python_parity.py
    ENV HELIX_ARBITER_REPLAY=$<TARGET_FILE:helix_arbiter_replay>
    TIMEOUT 300
  )

  # Both node executables as real processes on an isolated, localhost-only
  # DDS domain with synthetic topics (see the file header).
  ament_add_pytest_test(test_node_process test/test_node_process.py
    ENV HELIX_ARBITER_CPP_EXE=$<TARGET_FILE:helix_arbiter> ROS_DOMAIN_ID=91 ROS_LOCALHOST_ONLY=1
    TIMEOUT 600
  )
endif()

ament_package()
