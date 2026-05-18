cmake_minimum_required(VERSION 3.8)
project(gz_depth_republisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── ROS 2 dependencies ─────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── gz-transport (newest → oldest) ────────────────────────────────────────────
find_package(gz-transport13 QUIET)
if(gz-transport13_FOUND)
  set(GZ_TRANSPORT_TARGET gz-transport13::core)
  message(STATUS "gz-transport: using gz-transport13")
else()
  find_package(gz-transport12 QUIET)
  if(gz-transport12_FOUND)
    set(GZ_TRANSPORT_TARGET gz-transport12::core)
    message(STATUS "gz-transport: using gz-transport12")
  else()
    find_package(gz-transport REQUIRED)
    set(GZ_TRANSPORT_TARGET gz-transport::core)
    message(STATUS "gz-transport: using gz-transport (legacy)")
  endif()
endif()

# ── gz-msgs (newest → oldest) ─────────────────────────────────────────────────
find_package(gz-msgs10 QUIET)
if(gz-msgs10_FOUND)
  set(GZ_MSGS_TARGET gz-msgs10::core)
  message(STATUS "gz-msgs: using gz-msgs10")
else()
  find_package(gz-msgs9 QUIET)
  if(gz-msgs9_FOUND)
    set(GZ_MSGS_TARGET gz-msgs9::core)
    message(STATUS "gz-msgs: using gz-msgs9")
  else()
    find_package(gz-msgs REQUIRED)
    set(GZ_MSGS_TARGET gz-msgs::core)
    message(STATUS "gz-msgs: using gz-msgs (legacy)")
  endif()
endif()

# ── executable ─────────────────────────────────────────────────────────────────
add_executable(gz_depth_republisher_node
  src/gz_depth_republisher_node.cpp
)

target_include_directories(gz_depth_republisher_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(gz_depth_republisher_node
  rclcpp
  sensor_msgs
  std_msgs
)

target_link_libraries(gz_depth_republisher_node
  ${GZ_TRANSPORT_TARGET}
  ${GZ_MSGS_TARGET}
)

# ── install ────────────────────────────────────────────────────────────────────
install(TARGETS gz_depth_republisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include DESTINATION include)

# ── tests ──────────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
