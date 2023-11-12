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

/**
 * @file subscriber_member_function.cpp
 * @author Tej Kiran (itej89@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "cpp_pubsub_msgs/msg/tutorial_string.hpp"

using std::placeholders::_1;

/**
 * @brief ROS Node Class that demostrates the subscriber functionality
 * 
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    /**
     * @brief Create the subscription to the "topic"
     * 
     */
    subscription_ = this->create_subscription<cpp_pubsub_msgs::msg
    ::TutorialString>("custom_message", 10, std::bind(
                      &MinimalSubscriber::topic_callback, this, _1));
  }

 private:
 /**
  * @brief Create a callback for the topic
  * 
  * @param msg 
  */
  void topic_callback(const cpp_pubsub_msgs::msg::TutorialString& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.text.c_str());
  }

  /**
   * @brief Pointer for adding subscription
   * 
   */
  rclcpp::Subscription<cpp_pubsub_msgs::msg
    ::TutorialString>::SharedPtr subscription_;
};

/**
 * @brief Main funciton  for the subscriber node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
