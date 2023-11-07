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
 * @file publisher_member_function.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-11-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "cpp_pubsub_msgs/msg/tutorial_string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */


/**
 * @brief ROS demo publisher class node
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {

    /**
     * @brief Create a publisher to the topic "topic"
     * 
     */
    publisher_ = this->create_publisher<cpp_pubsub_msgs::msg::TutorialString>("custom_message", 10);

    /**
     * @brief Create timer to publish the message at periodic intervals
     * 
     */
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
 /**
  * @brief Timer callback that publishes messages periodically
  * 
  */
  void timer_callback() {

    /**
     * @brief Build the message
     * 
     */
    auto message = cpp_pubsub_msgs::msg::TutorialString();
    message.text = "With great power comes great responsibility!! " +
                   std::to_string(count_++);

    /**
     * @brief Publish the message
     * 
     */
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.text.c_str());
    publisher_->publish(message);
  }

  /**
   * @brief Timer parameter
   * 
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Pointer for the publisher
   * 
   */
  rclcpp::Publisher<cpp_pubsub_msgs::msg::TutorialString>::SharedPtr publisher_;

  /**
   * @brief Count to indientify the current message
   * 
   */
  size_t count_;
};


/**
 * @brief Main function for the publisher node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
