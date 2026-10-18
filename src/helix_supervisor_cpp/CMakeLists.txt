cmake_minimum_required(VERSION 3.8)
project(helix_supervisor_cpp)

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
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(helix_msgs REQUIRED)

# ROS-free fault-state machine.
add_library(helix_supervisor_core STATIC src/supervisor_core.cpp)
target_include_directories(helix_supervisor_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_target_properties(helix_supervisor_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(supervisor_component SHARED src/supervisor_node.cpp)
target_link_libraries(supervisor_component helix_supervisor_core)
ament_target_dependencies(supervisor_component
  rclcpp rclcpp_lifecycle rclcpp_components lifecycle_msgs std_msgs helix_msgs)
rclcpp_components_register_node(supervisor_component
  PLUGIN "helix_supervisor_cpp::SupervisorNode"
  EXECUTABLE helix_supervisor
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS supervisor_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_supervisor_core test/test_supervisor_core.cpp)
  target_link_libraries(test_supervisor_core helix_supervisor_core)

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_supervisor_node test/test_supervisor_node.py
    TIMEOUT 240)
endif()

ament_package()
