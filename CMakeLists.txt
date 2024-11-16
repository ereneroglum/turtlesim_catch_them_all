# turtlesim_catch_them_all Example project for learning ROS2
# Copyright (C) 2024  Eren Eroğlu
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
find_package(turtlesim REQUIRED)

# Build messages
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Turtle.msg)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# add turtle spawner binary to build list
add_executable(turtle_spawner
  include/turtlesim_catch_them_all/turtle_spawner.hpp
  src/spawner.cpp
  src/turtle_spawner.cpp)

target_include_directories(turtle_spawner
  PUBLIC include)

ament_target_dependencies(turtle_spawner rclcpp)
ament_target_dependencies(turtle_spawner turtlesim)

target_link_libraries(turtle_spawner "${cpp_typesupport_target}")

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
