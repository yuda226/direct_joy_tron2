cmake_minimum_required(VERSION 3.8)
project(direct_joy)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(direct_joy_node src/direct_joy_node.cpp)
target_compile_features(direct_joy_node PRIVATE cxx_std_17)
target_include_directories(direct_joy_node PRIVATE include)
ament_target_dependencies(direct_joy_node rclcpp sensor_msgs geometry_msgs std_srvs)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(direct_joy_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS direct_joy_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_test REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_gtest(test_controller test/test_controller.cpp)
  target_compile_features(test_controller PRIVATE cxx_std_17)
  target_include_directories(test_controller PRIVATE include)
  ament_target_dependencies(test_controller sensor_msgs geometry_msgs)
  ament_add_gtest(test_mode_buttons test/test_mode_buttons.cpp)
  target_compile_features(test_mode_buttons PRIVATE cxx_std_17)
  target_include_directories(test_mode_buttons PRIVATE include)
  ament_target_dependencies(test_mode_buttons sensor_msgs geometry_msgs)
  ament_add_test(test_ros_integration
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_ros_integration.py"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60)
endif()

ament_package()
