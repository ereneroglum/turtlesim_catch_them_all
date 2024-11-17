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

#include "turtlesim_catch_them_all/turtle_spawner.hpp"
#include "turtlesim_catch_them_all/srv/catch_turtle.hpp"
#include "turtlesim_catch_them_all/srv/detail/catch_turtle__struct.hpp"
#include <random>
#include <string>
#include <thread>
#include <turtlesim/srv/kill.hpp>

namespace turtlesim_catch_them_all {

static double randomDouble() {
  return double(std::rand()) / (double(RAND_MAX) + 1.0);
}

static constexpr double PI = 3.1415;

TurtleSpawner::TurtleSpawner(std::optional<std::string> name)
    : Node(name.value_or("turtle_spawner")) {
  this->m_name = name.value_or("turtle_spawner");
  m_randomTurtleSpawnTimer = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TurtleSpawner::spawnRandomTurtle, this));
  m_catchTurtleService =
      this->create_service<turtlesim_catch_them_all::srv::CatchTurtle>(
          "catch_turtle",
          [this](__attribute__((unused)) const turtlesim_catch_them_all::srv::
                     CatchTurtle::Request::SharedPtr _,
                 turtlesim_catch_them_all::srv::CatchTurtle::Response::SharedPtr
                     response) { response->ok = this->catchTurtle(); });
}

bool TurtleSpawner::catchTurtle() {
  if (this->m_turtles.empty()) {
    return false;
  }

  auto turtle = this->getTurtle();

  auto client = this->create_client<turtlesim::srv::Kill>("kill");
  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for Service Server to be up...");
  }

  auto request = std::make_shared<turtlesim::srv::Kill::Request>();
  request->name = turtle->name;

  auto future = client->async_send_request(request);

  try {
    future.get();
    this->m_turtles.pop();
    return true;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
    return false;
  }
}

void TurtleSpawner::spawnRandomTurtle() {
  double x = randomDouble() * 10;
  double y = randomDouble() * 10;
  double theta = randomDouble() * 2 * PI;
  auto thread = std::make_shared<std::thread>(
      std::thread([=]() { this->spawnTurtle(x, y, theta); }));
  m_spawnTurtleThreads.push_back(thread);
}

void TurtleSpawner::spawnTurtle(double x, double y, double theta,
                                std::optional<std::string> name) {
  // TODO: Remove auto types.
  auto client = this->create_client<turtlesim::srv::Spawn>("spawn");

  while (!client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Waiting for turtlesim service...");
  }

  auto request = std::make_shared<turtlesim::srv::Spawn_Request>();
  request->x = x;
  request->y = y;
  request->theta = theta;

  if (name != std::nullopt) {
    request->name = name.value();
  }

  auto future = client->async_send_request(request);

  try {
    auto response = future.get();
    if (response->name != "") {
      auto new_turtle = turtlesim_catch_them_all::msg::Turtle();
      new_turtle.name = response->name;
      new_turtle.x = x;
      new_turtle.y = y;
      new_turtle.theta = theta;
      this->m_turtles.push(new_turtle);
      RCLCPP_INFO(this->get_logger(), "Turtle %s is now alive.",
                  response->name.c_str());
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Service call failed.");
  }
}

std::optional<turtlesim_catch_them_all::msg::Turtle>
TurtleSpawner::getTurtle() {
  if (!this->m_turtles.empty()) {
    return std::make_optional(this->m_turtles.front());
  }
  return std::nullopt;
}

} // namespace turtlesim_catch_them_all
