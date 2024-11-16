cmake_minimum_required(VERSION 3.8)
project(turtlesim_catch_them_all)

# Export compile commands for completion via langauge server
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set C++ standard as C++ 17 with no compiler extensions
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Build messages
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Turtle.msg)

# add turtle spawner binary to build list
add_executable(turtle_spawner
  include/turtlesim_catch_them_all/turtle_spawner.hpp
  src/spawner.cpp
  src/turtle_spawner.cpp)

target_include_directories(turtle_spawner
  PUBLIC include)

ament_target_dependencies(turtle_spawner rclcpp)

# Use strict error checking
target_compile_options(turtle_spawner PRIVATE
  $<$<CXX_COMPILER_ID:MSVC>:/W4 /WX>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic -Werror>
)

install(TARGETS turtle_spawner)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
