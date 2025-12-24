cmake_minimum_required(VERSION 3.8)
project(manage_node)

# --- ROS2 基本依存 ---
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# --- OpenCV 追加 ---
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}           # ← /usr/include/opencv4 を自動追加
)

# --- ノード追加 ---
add_executable(manage_node src/manage_node.cpp)
ament_target_dependencies(manage_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
)

# --- OpenCV ライブラリリンク ---
target_link_libraries(manage_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  manage_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
