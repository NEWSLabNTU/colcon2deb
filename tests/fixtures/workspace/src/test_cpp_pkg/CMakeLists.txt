cmake_minimum_required(VERSION 3.8)
project(test_cpp_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(test_node src/test_node.cpp)
ament_target_dependencies(test_node rclcpp std_msgs)

install(TARGETS test_node DESTINATION lib/${PROJECT_NAME})

ament_package()
