/*
Copyright 2021 Scott Horton

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "speech_action_interfaces/action/recognize.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace speech_input_action_server
{
class RecognizeActionClient : public rclcpp::Node
{
public:
  using Recognize = speech_action_interfaces::action::Recognize;
  using GoalHandleRecognize = rclcpp_action::ClientGoalHandle<Recognize>;

  explicit RecognizeActionClient(const rclcpp::NodeOptions & options)
  : Node("recognize_action_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Recognize>(
      this,
      "recognize");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RecognizeActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = Recognize::Goal();
    goal_msg.timeout = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Recognize>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&RecognizeActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&RecognizeActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&RecognizeActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Recognize>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(const GoalHandleRecognize::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleRecognize::SharedPtr,
    const std::shared_ptr<const Recognize::Feedback> feedback)
  {
    std::stringstream ss;
    ss << "State: " << feedback->status;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleRecognize::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: " << result.result->text;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class RecognizeActionClient

}  // namespace speech_input_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(speech_input_action_server::RecognizeActionClient)
