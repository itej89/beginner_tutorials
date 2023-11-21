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
 * @version 0.2
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

#include "cpp_pubsub_msgs/msg/tutorial_string.hpp"
#include "cpp_pubsub_msgs/srv/tutorial_service.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "rclcpp/rclcpp.hpp"

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
    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initializing MinimalPublisher.");

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Create Publisher Instance");
    /**
     * @brief Create a publisher to the topic "topic"
     *
     */
    publisher_ = this->create_publisher<cpp_pubsub_msgs::msg ::TutorialString>(
        "custom_message", 10);

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Create Service Instance");
    /**
     * @brief Create a service client for "make_script" service
     *
     */
    service_client_ =
        this->create_client<cpp_pubsub_msgs::srv::TutorialService>(
            "make_script");

    /**
     * @brief Create a tf braodcaster instance
     * 
     */
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /**
     * @brief Declare the publisher rate as parameter
     *
     */
    this->declare_parameter("pub_rate", 200);

    /**
     * @brief Read the publish rate
     *
     */
    int pub_rate = this->get_parameter("pub_rate").as_int();

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Create Timer Instance");
    /**
     * @brief Create timer to publish the message at periodic intervals
     *
     */
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(pub_rate),
        std::bind(&MinimalPublisher::timer_callback, this));

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Calling movie dialogue service..");

    script_message = get_movie_dialogue(
        "Uncle Ben", "With great power comes great responsibility!!");

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Service Returned : " << script_message);

    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Initialization of MinimalPublisher done.");
  }

 private:
  /**
   * @brief Get the movie dialogue object
   *
   * @param _character character name
   * @param _dialogue  dialogue
   * @return std::string return sentence
   */
  std::string get_movie_dialogue(const std::string& _character,
                                 const std::string& _dialogue) {
    /**
     * @brief script line return value
     *
     */
    std::string script_line = "";

    /**
     * @brief wait for the service avaialbility
     *
     */
    if (!this->service_client_->wait_for_service(1s)) {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "service not available, skipping the message publish...");
    } else {
      /**
       * @brief create request
       *
       */
      auto request =
          std::make_shared<cpp_pubsub_msgs::srv::TutorialService::Request>();
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
          FutureReturnCode::SUCCESS) {
        script_line = result.get()->script;

        if (script_line == "") {
          RCLCPP_WARN_STREAM(this->get_logger(),
                             "Service returned empty result!!");
        }

        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                           "Script result: " << script_line);
      } else {
        RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"),
                            "Failed to call service movie_script");
      }
    }

    return script_line;
  }

  /**
   * @brief Method publishes tranform from /world to /talk
   * 
   */
  void publish_transform() {

    /**
     * @brief Create geometry message
     * 
     */
    geometry_msgs::msg::TransformStamped t;

    /**
     * @brief Set header frames and timestamp
     *  parent frame : /world
     *  child frame  : /talk
     */
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    /**
     * @brief Set translation as (10.0, 20.0, 0)
     * 
     */
    t.transform.translation.x = 10.0;
    t.transform.translation.y = 20.0;
    t.transform.translation.z = 0.0;

    /**
     * @brief Set rotation as 90 deg yaw
     * 
     */
    tf2::Quaternion q;
    q.setRPY(0, 0, 1.57 );
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    /**
     * @brief Send the transform
     * 
     */
    tf_broadcaster_->sendTransform(t);
  }

  /**
   * @brief Method publishes custom script message
   * 
   */
  void publish_message() {
        /**
     * @brief Build the message
     *
     */
    RCLCPP_DEBUG_STREAM(this->get_logger(),
                        "Publisher periodic timer callback hit!!");

    auto message = cpp_pubsub_msgs::msg::TutorialString();

    if (!this->service_client_->wait_for_service(100ms)) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "service not available, publishing default message...");
            message.text = "With great power comes great responsibility!! " + std::to_string(count_++);
            RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.text);
            publisher_->publish(message);
    } else {
      message.text = this->script_message + " " + std::to_string(count_++);

      /**
       * @brief Publish the message
       *
       */
      RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.text);
      publisher_->publish(message);
    }
  }

  /**
   * @brief Timer callback that publishes messages periodically
   *
   */
  void timer_callback() {

    /**
     * @brief Construct a new publish custom message
     * 
     */
    publish_message();

    /**
     * @brief Construct a new broadcast transform
     * 
     */
    publish_transform();

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
  rclcpp::Client<cpp_pubsub_msgs::srv::TutorialService>::SharedPtr
      service_client_;

  /**
   * @brief Pointer for the broadcaster
   * 
   */
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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

  std::shared_ptr<MinimalPublisher> node_shared =
      std::make_shared<MinimalPublisher>();

  rclcpp::spin(node_shared);
  rclcpp::shutdown();
  return 0;
}