#include "turtlesim_catch_them_all/turtle_spawner.hpp"

namespace turtlesim_catch_them_all {
TurtleSpawner::TurtleSpawner() : Node("turtle_spawner") {
  this->m_name = "turtle_spawner";
}

TurtleSpawner::TurtleSpawner(const std::string &name) : Node(name) {
  this->m_name = name;
}
} // namespace turtlesim_catch_them_all
