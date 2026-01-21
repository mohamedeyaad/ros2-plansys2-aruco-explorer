#include <memory>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

class MoveAction : public plansys2::ActionExecutorClient
{
public:
  MoveAction() : plansys2::ActionExecutorClient("move", 500ms) {
    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    
    // Coordinates with orientation (x, y, yaw_radians)
    waypoints_["wp1"] = {-6.0, -6.0, 3.14}; // 180 degrees
    waypoints_["wp2"] = {-6.0,  6.0, 1.57};  // 90 degrees
    waypoints_["wp3"] = { 6.0, -6.0, -1.57}; // -90 degrees
    waypoints_["wp4"] = { 6.0,  6.0, 0.0};  // 0 degrees
    waypoints_["wp_start"] = {0.0, 0.0, 0.0}; 
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) {
    auto args = get_arguments(); 
    std::string goal_wp = args[2]; 
    RCLCPP_INFO(get_logger(), "Moving to [%s]...", goal_wp.c_str());

    if (waypoints_.find(goal_wp) == waypoints_.end()) {
      finish(false, 0.0, "Waypoint not found");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
    
    if (!nav2_client_->wait_for_action_server(2s)) {
      finish(false, 0.0, "Nav2 offline");
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->now();
    goal_msg.pose.pose.position.x = waypoints_[goal_wp][0];
    goal_msg.pose.pose.position.y = waypoints_[goal_wp][1];
    
    tf2::Quaternion q;
    q.setRPY(0, 0, waypoints_[goal_wp][2]);
    goal_msg.pose.pose.orientation.x = q.x();
    goal_msg.pose.pose.orientation.y = q.y();
    goal_msg.pose.pose.orientation.z = q.z();
    goal_msg.pose.pose.orientation.w = q.w();

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) finish(true, 1.0, "Arrived");
      else finish(false, 0.0, "Failed");
    };

    nav2_client_->async_send_goal(goal_msg, send_goal_options);
    return ActionExecutorClient::on_activate(previous_state);
  }

private:
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
  std::map<std::string, std::vector<double>> waypoints_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "move"));
  node->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}