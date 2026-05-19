cmake_minimum_required(VERSION 3.8)
project(depth_to_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake        REQUIRED)
find_package(rclcpp             REQUIRED)
find_package(sensor_msgs        REQUIRED)
find_package(cv_bridge          REQUIRED)
find_package(message_filters    REQUIRED)
find_package(OpenCV             REQUIRED)

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(depth_to_pointcloud_node
  src/depth_to_pointcloud_node.cpp
)

ament_target_dependencies(depth_to_pointcloud_node
  rclcpp
  sensor_msgs
  cv_bridge
  message_filters
  OpenCV
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS depth_to_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL   # don't fail if dirs don't exist yet
)

ament_package()
