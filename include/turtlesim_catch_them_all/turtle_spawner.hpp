// turtlesim_catch_them_all Example project for learning ROS2
// Copyright (C) 2024  Eren Eroğlu
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef TURTLE_SPAWNER
#define TURTLE_SPAWNER

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace turtlesim_catch_them_all {
/**
 * @brief Turtle spawner node for spawning turtles on turtlesim
 *
 * @details This class is used to spawn turtles and store useful information
 * about spawned turtles.
 */
class TurtleSpawner : public rclcpp::Node {
public:
  /**
   * @brief Default constructor
   *
   * @details This is the default constructor for TurtleSpawner node. Since no
   * name is given for the node, defaults to "turtlespawner".
   */
  TurtleSpawner();

  /**
   * @brief Named constructor
   *
   * @details This constructor is functionally same as the default constructor
   * with the exception of accepting a name for the node.
   *
   * @param name[in] Name of the node.
   */
  TurtleSpawner(const std::string &name);

private:
  /**
   * @brief Node name
   *
   * @details This name will be used for announcing services and topics.
   * It should only be set from constructor.
   */
  std::string m_name;
};
} // namespace turtlesim_catch_them_all

#endif // TURTLE_SPAWNER_HPP
