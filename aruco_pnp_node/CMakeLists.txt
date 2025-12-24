cmake_minimum_required(VERSION 3.8)
project(aruco_pnp_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)   # ★ これを追加 ★

add_executable(aruco_pnp_node src/aruco_pnp_node.cpp)
ament_target_dependencies(aruco_pnp_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge              # ★ これを追加 ★
  OpenCV
)

install(TARGETS
  aruco_pnp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

