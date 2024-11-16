#include "turtlesim_catch_them_all/turtle_spawner.hpp"
#include <memory>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<turtlesim_catch_them_all::TurtleSpawner> node =
      std::make_shared<turtlesim_catch_them_all::TurtleSpawner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
