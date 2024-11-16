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

#include "turtlesim/srv/spawn.hpp"
#include "turtlesim_catch_them_all/msg/turtle.hpp"
#include <queue>
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
   * @param name[in] Name of the node
   */
  TurtleSpawner(const std::string &name);

  /**
   * @brief Get next turtle
   *
   * @details This function returns the first turtle on the queue without
   * removing the turtle if the queue is not empty. Otherwise it returns
   * std::nullopt.
   *
   * @return First turtle in the queue or std::nullopt.
   */
  std::optional<turtlesim_catch_them_all::msg::Turtle> getTurtle();

private:
  /**
   * @brief Node name
   *
   * @details This name will be used for announcing services and topics.
   * It should only be set from constructor.
   */
  std::string m_name;
  std::queue<turtlesim_catch_them_all::msg::Turtle> m_turtles;

  /**
   * @brief Send spawn request to turtlesim and record turtle
   *
   * @details This function sends spawn requests to turtlesim service. After a
   * successfull call, it records the information about turtle.
   *
   * @param x[in] Location of turtle on the x axis
   * @param y[in] Location of turtle on the y axis
   * @param theta[in] Orientation of turtle
   * @param name[in] (Optional) Name of the turtle. Defaults to std::nullopt
   * which means it is up to the upstream service to decide the name.
   *
   * @note For internal use only
   */
  void spawnTurtle(double x, double y, double theta,
                   std::optional<std::string> name = std::nullopt);
};
} // namespace turtlesim_catch_them_all

#endif // TURTLE_SPAWNER_HPP
