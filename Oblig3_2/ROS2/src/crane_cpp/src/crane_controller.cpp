// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "crane_interfaces/msg/crane_reference.hpp" //CHANGE
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("crane_controll"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10); // CHANGE
    timer_ = this->create_wall_timer(50ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();		// CHANGE
    message.name = {"base_to_crane_boom"};					// CHANGE
    message.position = {angle};
    //message.velocity = {0.01}; 
    message.header.stamp = this->get_clock()->now();
    //message.crane_cylinder_velocity_reference = std::to_float(count_++); // CHANGE
    RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.position[0]);	// CHANGE'
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.velocity[0]);	// CHANGE
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.name[0].c_str());	// CHANGE
    publisher_->publish(message);
    
    time = time + 0.01;
    angle = hight*sin(time);
    
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;	// CHANGe
  size_t count_;
  float time = 0.0;
  float angle = 0.0;
  float hight = 0.4;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
