cmake_minimum_required(VERSION 3.8)
project(lidar_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ---- PCL ----
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(scan_shadow_filter_node
  src/scan_shadow_filter_node.cpp
)
target_compile_features(scan_shadow_filter_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(scan_shadow_filter_node
  rclcpp sensor_msgs std_msgs geometry_msgs visualization_msgs
)

add_executable(lidar_scan_matcher_node
  src/lidar_scan_matcher_node.cpp
)
target_compile_features(lidar_scan_matcher_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(lidar_scan_matcher_node
  rclcpp sensor_msgs std_msgs geometry_msgs pcl_conversions
)

target_link_libraries(lidar_scan_matcher_node
  ${PCL_LIBRARIES}
)

install(TARGETS
  scan_shadow_filter_node
  lidar_scan_matcher_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
