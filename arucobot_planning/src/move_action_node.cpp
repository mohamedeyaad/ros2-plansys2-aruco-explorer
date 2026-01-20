#include <memory>
#include <algorithm>
#include <map>
#include <string>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp" // New: To report ID
#include "aruco_interfaces/msg/aruco_markers.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

// ==========================================
// 1. Move Action (Navigation)
// ==========================================
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
    RCLCPP_INFO(get_logger(), "Navigating to [%s]...", goal_wp.c_str());

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
    
    // Convert yaw (radians) to quaternion
    double yaw = waypoints_[goal_wp][2];
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);  // Roll=0, Pitch=0, Yaw=specified
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

// ==========================================
// 2. Survey Action (Phase 1: Just Scan & Report)
// ==========================================
class SurveyAction : public plansys2::ActionExecutorClient
{
public:
  SurveyAction() : plansys2::ActionExecutorClient("survey", 100ms) {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    // Publish found ID to this topic
    id_pub_ = this->create_publisher<std_msgs::msg::Int32>("/found_id", 10); 
    
    aruco_sub_ = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
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

  void aruco_callback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
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
         cmd_vel_pub_->publish(cmd);
         finish(true, 1.0, "Survey done (ID Found)");
         return;
      }
    }
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr id_pub_;
  rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_sub_;
  rclcpp::Time start_time_;
};

// ==========================================
// 3. Take Picture Action (Phase 2: Center & Click)
// ==========================================
class TakePictureAction : public plansys2::ActionExecutorClient
{
public:
  TakePictureAction() : plansys2::ActionExecutorClient("take_picture", 100ms) {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    processed_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/modified_image", 10);
    
    aruco_sub_ = this->create_subscription<aruco_interfaces::msg::ArucoMarkers>(
      "/aruco_markers", rclcpp::SensorDataQoS(), 
      std::bind(&TakePictureAction::aruco_callback, this, std::placeholders::_1));

    img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", rclcpp::SensorDataQoS(), 
      [this](const sensor_msgs::msg::Image::SharedPtr msg){ last_image_msg_ = msg; });
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state) {
    RCLCPP_INFO(get_logger(), "ACTION: Locating for Picture...");
    state_ = SCANNING;
    start_time_ = this->now();
    return ActionExecutorClient::on_activate(previous_state);
  }

  void do_work() {
    geometry_msgs::msg::Twist cmd;
    if(state_ == SCANNING) {
        cmd.angular.z = 0.5;
        cmd_vel_pub_->publish(cmd);
        if((this->now() - start_time_).seconds() > 30.0) {
            finish(true, 1.0, "Timeout");
        }
    } else if (state_ == CENTERING) {
        if(std::abs(marker_x_) < 0.05) {
            cmd.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd);
            state_ = CAPTURING;
        } else {
            cmd.angular.z = std::max(std::min(-0.6 * marker_x_, 0.5), -0.5);
            cmd_vel_pub_->publish(cmd);
        }
    } else if (state_ == CAPTURING) {
      if (last_image_msg_) {
        try {
          // 1. Convert ROS message to OpenCV Mat
          cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(last_image_msg_, sensor_msgs::image_encodings::BGR8);
                
          // 2. Setup ArUco Detector (C++ equivalent of your Python setup)
          // Note: Ensure this dictionary matches what your robot uses (e.g. DICT_5X5_250, ORIGINAL, etc.)
          auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
          auto parameters = cv::aruco::DetectorParameters::create();
                
          std::vector<int> markerIds;
          std::vector<std::vector<cv::Point2f>> markerCorners;

          // 3. Detect Markers
          cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, markerIds, parameters);

          // 4. Find the specific target ID and draw
          for (size_t i = 0; i < markerIds.size(); i++) {
            if (markerIds[i] == target_id_) {
              // Get corners for this marker
              const auto& c = markerCorners[i]; // c[0]=TL, c[1]=TR, c[2]=BR, c[3]=BL
                        
              // Calculate Centroid (Average of Top-Left and Bottom-Right)
              // In C++, we cast to int for pixel coordinates
              int cx = static_cast<int>((c[0].x + c[2].x) / 2.0);
              int cy = static_cast<int>((c[0].y + c[2].y) / 2.0);

              // Draw Green Circle
              cv::circle(cv_ptr->image, cv::Point(cx, cy), 50, cv::Scalar(0, 0, 255), 3);

              // Draw Text
              std::string text = "ID: " + std::to_string(markerIds[i]);
              cv::putText(cv_ptr->image, text, cv::Point(cx - 30, cy - 60),
                    cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
            }
          }

          // 5. Publish Result
          processed_img_pub_->publish(*cv_ptr->toImageMsg());
                
          finish(true, 1.0, "Picture Taken");
            
        } catch (cv_bridge::Exception& e) {
          RCLCPP_ERROR(get_logger(), "CV Bridge Error: %s", e.what());
        }
      }
    }
  }

  void aruco_callback(const aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
    if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) return;
    
    if (state_ == SCANNING) {
        for(size_t i=0; i<msg->marker_ids.size(); i++) {
            if(msg->poses[i].position.z < 3.0) {
                target_id_ = msg->marker_ids[i];
                marker_x_ = msg->poses[i].position.x;
                state_ = CENTERING;
                geometry_msgs::msg::Twist cmd; cmd_vel_pub_->publish(cmd); // stop
                return;
            }
        }
    } else if (state_ == CENTERING) {
        for(size_t i=0; i<msg->marker_ids.size(); i++) {
            if(msg->marker_ids[i] == target_id_) {
                marker_x_ = msg->poses[i].position.x;
                return;
            }
        }
    }
  }

private:
  enum State { SCANNING, CENTERING, CAPTURING };
  State state_;
  double marker_x_;
  long target_id_;
  rclcpp::Time start_time_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr processed_img_pub_;
  rclcpp::Subscription<aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  sensor_msgs::msg::Image::SharedPtr last_image_msg_;
};

// ==========================================
// MAIN
// ==========================================
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  auto move_node = std::make_shared<MoveAction>();
  auto survey_node = std::make_shared<SurveyAction>();
  auto pic_node = std::make_shared<TakePictureAction>();

  move_node->set_parameter(rclcpp::Parameter("action_name", "move"));
  survey_node->set_parameter(rclcpp::Parameter("action_name", "survey"));
  pic_node->set_parameter(rclcpp::Parameter("action_name", "take_picture"));

  move_node->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  survey_node->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  pic_node->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(move_node->get_node_base_interface());
  executor.add_node(survey_node->get_node_base_interface());
  executor.add_node(pic_node->get_node_base_interface());

  executor.spin();
  rclcpp::shutdown();
  return 0;
}