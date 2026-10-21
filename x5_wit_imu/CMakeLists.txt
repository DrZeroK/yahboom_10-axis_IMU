cmake_minimum_required(VERSION 3.8)
project(x5_wit_imu)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(wit_imu_node_lib STATIC src/wit_imu_node.cpp)
target_include_directories(wit_imu_node_lib PUBLIC include)
ament_target_dependencies(wit_imu_node_lib rclcpp sensor_msgs diagnostic_msgs)

add_executable(wit_imu_node src/main.cpp)
target_link_libraries(wit_imu_node wit_imu_node_lib)

install(TARGETS wit_imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  # 协议层测试与对拍工具不需要 ROS；节点测试用 pty 假串口
  add_executable(test_wit_protocol test/test_wit_protocol.cpp)
  target_include_directories(test_wit_protocol PRIVATE include)
  add_test(NAME test_wit_protocol COMMAND test_wit_protocol)

  add_executable(wit_replay test/replay.cpp)
  target_include_directories(wit_replay PRIVATE include)

  add_executable(test_wit_imu_node test/test_wit_imu_node.cpp)
  target_link_libraries(test_wit_imu_node wit_imu_node_lib util)
  add_test(NAME test_wit_imu_node COMMAND test_wit_imu_node)
endif()

ament_package()
