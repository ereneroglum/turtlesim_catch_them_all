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

namespace turtlesim_catch_them_all {
TurtleSpawner::TurtleSpawner() : Node("turtle_spawner") {
  this->m_name = "turtle_spawner";
}

TurtleSpawner::TurtleSpawner(const std::string &name) : Node(name) {
  this->m_name = name;
}
} // namespace turtlesim_catch_them_all
