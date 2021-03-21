#include <functional>
#include <memory>

#include "speech_action_interfaces/action/recognize.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "speech_input_proc.h"

namespace speech_input_action_server
{
class SpeechInputActionServer : public rclcpp::Node
{
public:
  using Recognize = speech_action_interfaces::action::Recognize;
  using GoalHandleRecognize = rclcpp_action::ServerGoalHandle<Recognize>;


  explicit SpeechInputActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("speech_input_action_server", options),
	speech_proc_(NULL)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Recognize>(
      this,
      "recognize",
      std::bind(&SpeechInputActionServer::handle_goal, this, _1, _2),
      std::bind(&SpeechInputActionServer::handle_cancel, this, _1),
      std::bind(&SpeechInputActionServer::handle_accepted, this, _1));

    this->vad_publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    speech_proc_ = new SpeechInputProc();
    speech_proc_->Open();
    speech_proc_->WakeWordEnable(SpeechInputProc::WakeWordDetector_HeyRobot, true);
    speech_proc_->SetWakeWordCB(std::bind(&SpeechInputActionServer::wake_word_detected, this, _1));
    speech_proc_->SetRecogizeCB(std::bind(&SpeechInputActionServer::speech_recog_finished, this, _1, _2));
    speech_proc_->SetVADCB(std::bind(&SpeechInputActionServer::voice_detect_change, this, _1));
    speech_proc_->SetAOACB(std::bind(&SpeechInputActionServer::angle_of_arrival_change, this, _1));
  }

  void wake_word_detected(std::string wake_word)
  {
	  RCLCPP_INFO(this->get_logger(), "Wake word detected: %s", wake_word.c_str());
  }

  void speech_recog_finished(SpeechProcStatus status, std::string text)
  {
	  if (status == SpeechProcStatus_Ok) {
		  RCLCPP_INFO(this->get_logger(), "Speech recognized:  %s", text.c_str());
		  auto result = std::make_shared<Recognize::Result>();
		  result->text = text;
		  goal_handle_->succeed(result);
		  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
	  } else {
		  RCLCPP_INFO(this->get_logger(), "Error recognizing speech");
	  }
  }

  void voice_detect_change(bool voice_detected)
  {
	  RCLCPP_INFO(this->get_logger(), "Voice detected: %d", voice_detected);
  }

  void angle_of_arrival_change(int32_t angle)
  {
	  RCLCPP_INFO(this->get_logger(), "Angle of Arrival: %d", angle);
  }

private:
  rclcpp_action::Server<Recognize>::SharedPtr action_server_;

  std::shared_ptr<GoalHandleRecognize> goal_handle_;

  SpeechInputProc *speech_proc_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Recognize::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with timeout %d", goal->timeout);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRecognize> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleRecognize> goal_handle)
  {
    using namespace std::placeholders;
    goal_handle_ = goal_handle;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    //std::thread{std::bind(&SpeechInputActionServer::execute, this, _1), goal_handle}.detach();
//    auto result = std::make_shared<Recognize::Result>();
    if (speech_proc_->RecognizeStart() == SpeechProcStatus_Error) {
	  auto result = std::make_shared<Recognize::Result>();
	  result->text = "ERROR";
	  goal_handle_->succeed(result);
	  RCLCPP_INFO(this->get_logger(), "Goal failed to start");
    }
//    result->text = "blah blah";
//    goal_handle->succeed(result);
//    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }

};  // class SpeechInputActionServer

}  // namespace speech_input_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(speech_input_action_server::SpeechInputActionServer)
