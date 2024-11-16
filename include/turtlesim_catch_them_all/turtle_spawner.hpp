#ifndef TURTLE_SPAWNER
#define TURTLE_SPAWNER

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace turtlesim_catch_them_all {
/**
 * @brief Turtle spawner node for spawning turtles on turtlesim.
 *
 * @details This class is used to spawn turtles and store useful information
 * about spawned turtles.
 */
class TurtleSpawner : public rclcpp::Node {
public:
  /**
   * @brief Default constructor.
   *
   * @detail This is the default constructor for TurtleSpawner node. Since no
   * name is given for the node, defaults to "turtlespawner"
   */
  TurtleSpawner();

  /**
   * @brief Named constructor.
   *
   * @detail This constructor is functionally same as the default constructor
   * with the exception of accepting a name for the node.
   *
   * @param name[in] Name of the node.
   */
  TurtleSpawner(const std::string &name);

private:
  std::string m_name;
};
} // namespace turtlesim_catch_them_all

#endif // TURTLE_SPAWNER_HPP
