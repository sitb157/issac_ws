cmake_minimum_required(VERSION 3.5)
project(isaac_moveit)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit REQUIRED)
find_package(moveit_resources_panda_moveit_config REQUIRED)
find_package(moveit_ros_move_group REQUIRED)

find_package(control_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)

find_package(pluginlib REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  rviz2
  DESTINATION share/${PROJECT_NAME})

ament_package()
