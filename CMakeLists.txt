cmake_minimum_required(VERSION 3.8)
project(depth_to_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake     REQUIRED)
find_package(rclcpp          REQUIRED)
find_package(sensor_msgs     REQUIRED)
find_package(gz-transport13  REQUIRED)   # adjust version to your Gazebo (Harmonic=8, Ionic=13, etc.)
find_package(gz-msgs10       REQUIRED)   # matches gz-transport version

# ── Executable ────────────────────────────────────────────────────────────────
add_executable(depth_to_pointcloud_node
  src/depth_to_pointcloud_node.cpp
)
target_compile_options(depth_to_pointcloud_node PRIVATE -O3 -march=native)

ament_target_dependencies(depth_to_pointcloud_node
  rclcpp
  sensor_msgs
)

# gz-transport and gz-msgs are not ament packages; link manually
target_link_libraries(depth_to_pointcloud_node
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS depth_to_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()