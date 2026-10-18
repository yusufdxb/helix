cmake_minimum_required(VERSION 3.8)
project(helix_go2_sink_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(helix_msgs REQUIRED)
# The node needs the robot's message packages (unitree_ros2). Without them
# (CI) only the core, the replay tool and the tests are built.
find_package(unitree_api QUIET)
find_package(unitree_go QUIET)

# ROS-free decision logic and feedback tracking.
add_library(helix_sink_core STATIC src/sink_core.cpp)
target_include_directories(helix_sink_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Vendor-independent experimental last-hop protection. Its node is isolated
# from native launch and permits non-dry operation only on localhost topics.
add_library(helix_sport_guard_core STATIC src/sport_guard_core.cpp)
target_link_libraries(helix_sport_guard_core PUBLIC helix_sink_core)
if(unitree_api_FOUND)
  add_executable(helix_sport_guard src/sport_guard_node.cpp)
  target_link_libraries(helix_sport_guard helix_sport_guard_core)
  ament_target_dependencies(helix_sport_guard rclcpp std_msgs helix_msgs unitree_api)
  install(TARGETS helix_sport_guard DESTINATION lib/${PROJECT_NAME})
endif()

if(unitree_api_FOUND AND unitree_go_FOUND)
  add_executable(helix_go2_sport_sink src/sink_node.cpp)
  target_link_libraries(helix_go2_sport_sink helix_sink_core)
  ament_target_dependencies(helix_go2_sport_sink
    rclcpp std_msgs geometry_msgs helix_msgs unitree_api unitree_go)
  install(TARGETS helix_go2_sport_sink DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "unitree_api/unitree_go not found: helix_go2_sport_sink is not built")
endif()

# Event replay for the Python parity test.
add_executable(helix_sink_replay tools/sink_replay.cpp)
target_link_libraries(helix_sink_replay helix_sink_core)

install(TARGETS helix_sink_replay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_sink_core test/test_sink_core.cpp)
  target_link_libraries(test_sink_core helix_sink_core)

  ament_add_gtest(test_sport_guard_core test/test_sport_guard_core.cpp)
  target_link_libraries(test_sport_guard_core helix_sport_guard_core)

  ament_add_pytest_test(test_python_parity test/test_python_parity.py
    ENV HELIX_SINK_REPLAY=$<TARGET_FILE:helix_sink_replay>
    TIMEOUT 120
  )
endif()

ament_package()
