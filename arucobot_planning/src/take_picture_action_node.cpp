#include <memory>
#include "plansys2_executor/ActionExecutorClient.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "arucobot_interfaces/msg/aruco_markers.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace std::chrono_literals;

class TakePictureAction : public plansys2::ActionExecutorClient
{
public:
  TakePictureAction() : plansys2::ActionExecutorClient("take_picture", 100ms) {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    processed_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/modified_image", 10);
    
    aruco_sub_ = this->create_subscription<arucobot_interfaces::msg::ArucoMarkers>(
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
        if((this->now() - start_time_).seconds() > 30.0) finish(true, 1.0, "Timeout");
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
          
          // 2. Setup ArUco Detector
          auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
          auto parameters = cv::aruco::DetectorParameters::create();
          
          std::vector<int> markerIds;
          std::vector<std::vector<cv::Point2f>> markerCorners;

          // 3. Detect Markers
          cv::aruco::detectMarkers(cv_ptr->image, dictionary, markerCorners, markerIds, parameters);
          
          // 4. Find the specific target ID and draw
          for (size_t i = 0; i < markerIds.size(); i++) {
             if (markerIds[i] == target_id_) {
                const auto& c = markerCorners[i]; // c[0]=TL, c[1]=TR, c[2]=BR, c[3]=BL
                int cx = static_cast<int>((c[0].x + c[2].x) / 2.0);
                int cy = static_cast<int>((c[0].y + c[2].y) / 2.0);
                // Draw Red Circle
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
        } catch (cv_bridge::Exception& e) {}
      }
    }
  }

  void aruco_callback(const arucobot_interfaces::msg::ArucoMarkers::SharedPtr msg) {
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
  rclcpp::Subscription<arucobot_interfaces::msg::ArucoMarkers>::SharedPtr aruco_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr img_sub_;
  sensor_msgs::msg::Image::SharedPtr last_image_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TakePictureAction>();
  node->set_parameter(rclcpp::Parameter("action_name", "take_picture"));
  node->trigger_transition(rclcpp_lifecycle::Transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE));
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}