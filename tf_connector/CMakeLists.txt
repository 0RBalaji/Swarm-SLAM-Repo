cmake_minimum_required(VERSION 3.8)
project(tf_connector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)       # For ROS 2 C++ APIs
find_package(tf2_ros REQUIRED)      # For Transform Broadcaster
find_package(tf2 REQUIRED)          # For Transform Utilities
find_package(geometry_msgs REQUIRED) # For geometry messages
find_package(nav_msgs REQUIRED)

# Declare the executable
add_executable(worldframe_game src/worldframe_game.cpp)
add_executable(multi_tf_bridge src/multi_tf_bridge.cpp)

# Link dependencies for worldframe_game
ament_target_dependencies(worldframe_game
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  nav_msgs
)

# # Link dependencies for multi_tf_bridge
ament_target_dependencies(multi_tf_bridge
  geometry_msgs
  rclcpp
  tf2
  tf2_ros
  nav_msgs
)

# Install the executable
install(
  TARGETS worldframe_game multi_tf_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# Install additional directories
install(
  DIRECTORY launch include src
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
