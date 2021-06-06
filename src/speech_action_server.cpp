#include <functional>
#include <memory>

#include "speech_action_interfaces/action/recognize.hpp"
#include "speech_action_interfaces/msg/wakeword.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
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
	speech_proc_(NULL),
	speaking_(false)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Recognize>(
      this,
      "recognize",
      std::bind(&SpeechInputActionServer::handle_goal, this, _1, _2),
      std::bind(&SpeechInputActionServer::handle_cancel, this, _1),
      std::bind(&SpeechInputActionServer::handle_accepted, this, _1));

    listening_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/speech_detect/listening", 2);
    vad_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/speech_detect/vad", 2);
    aoa_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/speech_detect/aoa", 2);
    ww_publisher_ = this->create_publisher<speech_action_interfaces::msg::Wakeword>("/speech_detect/wakeword", 2);

    speech_active_sub_ = this->create_subscription<std_msgs::msg::Bool>("/head/speaking",
    	rclcpp::SystemDefaultsQoS(),
		std::bind(&SpeechInputActionServer::speakingActiveCB, this, std::placeholders::_1));


    RCLCPP_INFO(this->get_logger(), "Initializing speech input processor");
    speech_proc_ = new SpeechInputProc();
    speech_proc_->Open();
    speech_proc_->WakeWordEnable(SpeechInputProc::WakeWordDetector_HeyRobot, true);
    speech_proc_->WakeWordEnable(SpeechInputProc::WakeWordDetector_HeyAnna, true);
    speech_proc_->SetWakeWordCB(std::bind(&SpeechInputActionServer::wake_word_detected, this, _1));
    speech_proc_->SetRecogizeCB(std::bind(&SpeechInputActionServer::speech_recog_finished, this, _1, _2));
    speech_proc_->SetVADCB(std::bind(&SpeechInputActionServer::voice_detect_change, this, _1));
    speech_proc_->SetAOACB(std::bind(&SpeechInputActionServer::angle_of_arrival_change, this, _1));
  }

  void wake_word_detected(std::string wake_word)
  {
	  RCLCPP_INFO(this->get_logger(), "Wake word detected: %s", wake_word.c_str());
	  auto message = speech_action_interfaces::msg::Wakeword();
	  message.stamp = this->now();
	  message.word = wake_word;
	  ww_publisher_->publish(message);
  }

  void speech_recog_finished(SpeechProcStatus status, std::string text)
  {
	  auto result = std::make_shared<Recognize::Result>();
	  if (status == SpeechProcStatus_Ok) {
		  RCLCPP_INFO(this->get_logger(), "Speech recognized:  %s", text.c_str());
		  result->text = text;
		  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
	  } else {
		  RCLCPP_INFO(this->get_logger(), "Error recognizing speech");
		  result->text = "ERROR";
	  }
	  goal_handle_->succeed(result);
  }

  void listening_change(bool listening)
  {
	  RCLCPP_INFO(this->get_logger(), "Change in listening state: %d", listening);
	  auto message = std_msgs::msg::Bool();
	  message.data = listening;
	  vad_publisher_->publish(message);
  }

  void voice_detect_change(bool voice_detected)
  {
	  RCLCPP_INFO(this->get_logger(), "Voice detected: %d", voice_detected);
	  if (!speaking_) {
		  auto message = std_msgs::msg::Bool();
		  message.data = voice_detected;
		  vad_publisher_->publish(message);
	  }
  }

  void angle_of_arrival_change(int32_t angle)
  {
	  RCLCPP_INFO(this->get_logger(), "Angle of Arrival: %d", angle);
	  if (!speaking_) {
		  auto message = std_msgs::msg::Int32();
		  message.data = angle;
		  aoa_publisher_->publish(message);
	  }
  }

  void speakingActiveCB(std_msgs::msg::Bool::SharedPtr msg)
  {
	  RCLCPP_INFO(this->get_logger(), "Speaking active msg: %d", msg->data);
	  speech_proc_->MuteInput(msg->data);
	  speaking_ = msg->data;
  }

private:
  rclcpp_action::Server<Recognize>::SharedPtr action_server_;

  std::shared_ptr<GoalHandleRecognize> goal_handle_;

  SpeechInputProc *speech_proc_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr listening_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vad_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr aoa_publisher_;
  rclcpp::Publisher<speech_action_interfaces::msg::Wakeword>::SharedPtr ww_publisher_;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr speech_active_sub_;

  bool speaking_;

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
    const auto goal = goal_handle->get_goal();

    if (speech_proc_->RecognizeStart() == SpeechProcStatus_Error) {
	  auto result = std::make_shared<Recognize::Result>();
	  result->text = "ERROR";
	  goal_handle_->succeed(result);
	  RCLCPP_INFO(this->get_logger(), "Goal failed to start");
    }
  }

};  // class SpeechInputActionServer

}  // namespace speech_input_action_server

RCLCPP_COMPONENTS_REGISTER_NODE(speech_input_action_server::SpeechInputActionServer)
