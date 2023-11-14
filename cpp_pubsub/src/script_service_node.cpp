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
 * @file script_service_node.cpp
 * @author Tej Kiran (itej89@gmail.com)
 * @brief This file contains an example node that demostrates the service
 * functionality of ROS2
 * @version 0.1
 * @date 2023-11-07
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <memory>

#include "cpp_pubsub_msgs/srv/tutorial_service.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @brief the call back funciton for the service
 * 
 * @param request {consists of two inputs string}
 *  1. character : name of the character 
 *  2. dialogue  : dialogue of the character
 * @param response {consists of one output string}
 *  1. script : <character>+" told : "+<dialogue>
 */
void make_script(
    const std::shared_ptr<cpp_pubsub_msgs::srv::TutorialService::Request>
        request,
    std::shared_ptr<cpp_pubsub_msgs::srv::TutorialService::Response> response) {

  /**
   * @brief generate the script line
   * 
   */
  response->script = request->character + " told : " + request->dialogue;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Incoming request\ncharacter: %s"
              " dialogue: %s",
              request->character.c_str(), request->dialogue.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back script line: [%s]",
              response->script.c_str());
}

/**
 * @brief Main funcction for the server node
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  /**
   * @brief Initialize the node
   * 
   */
  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("make_script_server");

/**
 * @brief Initialize service inside the node
 * 
 */
  rclcpp::Service<cpp_pubsub_msgs::srv::TutorialService>::SharedPtr
      make_script_service =
          node->create_service<cpp_pubsub_msgs::srv::TutorialService>(
              "make_script", &make_script);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
              "Ready to create movie script lines.");

  /**
   * @brief spin the ROS2
   * 
   */
  rclcpp::spin(node);
  rclcpp::shutdown();
}