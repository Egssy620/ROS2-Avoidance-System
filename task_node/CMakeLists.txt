cmake_minimum_required(VERSION 3.8)
project(task_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(task_node src/task_node.cpp)
ament_target_dependencies(task_node rclcpp std_msgs geometry_msgs)

install(TARGETS
  task_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

