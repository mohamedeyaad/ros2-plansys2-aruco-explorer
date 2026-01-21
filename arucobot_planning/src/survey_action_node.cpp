#include <memory>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "arucobot_interfaces/msg/aruco_markers.hpp"

using namespace std::chrono_literals;

class SurveyAction : public plansys2::ActionExecutorClient
{
public:
  SurveyAction() : plansys2::ActionExecutorClient("survey", 100ms) {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // Publish found ID to this topic
    id_pub_ = this->create_publisher<std_msgs::msg::Int32>("/found_id", 10); 
    aruco_sub_ = this->create_subscription<arucobot_interfaces::msg::ArucoMarkers>(
      "/aruco_markers", rclcpp::SensorDataQoS(), 
      std::bind(&SurveyAction::aruco_callback, this, std::placeholders::_1));
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) {
    RCLCPP_INFO(get_logger(), "SURVEY: Spinning to find ID...");
    start_time_ = this->now();
    return ActionExecutorClient::on_activate(previous_state);
  }

  void do_work() {
    auto now = this->now();
    // Spin logic
    geometry_msgs::msg::Twist cmd;
    cmd.angular.z = 0.5;
    cmd_vel_pub_->publish(cmd);

    // Timeout after 30s
    if ((now - start_time_).seconds() > 30.0) {
       cmd.angular.z = 0.0;
       cmd_vel_pub_->publish(cmd);
       finish(true, 1.0, "Survey done (No ID found)");
    }
  }

  void aruco_callback(const arucobot_interfaces::msg::ArucoMarkers::SharedPtr msg) {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    if (msg->marker_ids.empty()) return;

    // Find closest valid marker
    for (size_t i = 0; i < msg->marker_ids.size(); i++) {
      if (msg->poses[i].position.z < 3.0) { // < 3m filter
         // Report ID
         std_msgs::msg::Int32 id_msg;
         id_msg.data = msg->marker_ids[i];
         id_pub_->publish(id_msg);
         RCLCPP_INFO(get_logger(), "SURVEY: Found ID %d", id_msg.data);
         
         // Stop and Finish
         geometry_msgs::msg::Twist cmd;
         cmd_vel_pub_->publish(cmd); // Stop
         finish(true, 1.0, "Survey done (ID Found)");
         return;
      }
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_pub_;
  rclcpp::Subscription<arucobot_interfaces::msg::ArucoMarkers>::SharedPtr aruco_sub_;
  rclcpp::Time start_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SurveyAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "survey"));
  node->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}