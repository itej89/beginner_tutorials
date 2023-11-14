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
 * @author Tej Kiran (itej89@gmail.com)
 * @brief This file contains an example node that demostrates the publisher
 * functionality of ROS2
 * @version 0.1
 * @date 2023-11-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/client.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "cpp_pubsub_msgs/msg/tutorial_string.hpp"
#include "cpp_pubsub_msgs/srv/tutorial_service.hpp"


using namespace std::chrono_literals;

using namespace rclcpp;

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
    publisher_ = this->create_publisher<cpp_pubsub_msgs::msg ::TutorialString>(
        "custom_message", 10);


    /**
     * @brief Create a service client for "make_script" sedvice
     * 
     */
    service_client_ = this->create_client<cpp_pubsub_msgs::srv::TutorialService>("make_script");


    /**
     * @brief Create timer to publish the message at periodic intervals
     *
     */
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));


    script_message = get_movie_dialogue("Uncle Ben", 
      "With great power comes great responsibility!!");
  }

 private:

  /**
   * @brief Get the movie dialogue object
   * 
   * @param _character character name
   * @param _dialogue  dialogue
   * @return std::string return sentence
   */
  std::string get_movie_dialogue(std::string _character, std::string _dialogue) {

    /**
     * @brief script line return value
     * 
     */
    std::string script_line = "";

    /**
     * @brief wait for the service avaialbility
     * 
     */
    if (!this->service_client_->wait_for_service(4s)) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, skipping the message publish...");
    }
    else
    { 
      /**
       * @brief create request
       * 
       */
      auto request = std::make_shared<cpp_pubsub_msgs::srv::TutorialService::Request>();
      request->character = _character;
      request->dialogue = _dialogue;

      /**
       * @brief Call the service
       * 
       */
      auto result = this->service_client_->async_send_request(request);
      
      /**
       * @brief Wait for the result
       * 
       */
      if (spin_until_future_complete(this->get_node_base_interface(), result) ==
        FutureReturnCode::SUCCESS)
      {
        script_line = result.get()->script;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Script result: %s", script_line.c_str());
      }
      else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service movie_script");
      }
    }

    return script_line;
  }

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

    if (!this->service_client_->wait_for_service(1s)) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, skipping the message publish...");
    }
    else
    {
      message.text = this->script_message + " " +
                   std::to_string(count_++);

      /**
       * @brief Publish the message
       *
       */
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.text.c_str());
      publisher_->publish(message);
    }
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
   * @brief Pointer for the publisher
   *
   */
  rclcpp::Client<cpp_pubsub_msgs::srv::TutorialService>::SharedPtr service_client_;


  /**
   * @brief Count to indientify the current message
   *
   */
  size_t count_;

  /**
   * @brief Stores the script line obtained from the service
   * 
   */
  std::string script_message = "";
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
  
  std::shared_ptr<MinimalPublisher> node_shared  
            = std::make_shared<MinimalPublisher>();

  rclcpp::spin(node_shared);    
  rclcpp::shutdown();
  return 0;
}
