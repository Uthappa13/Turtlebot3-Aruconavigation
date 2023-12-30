#include "navigate.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


// function to listen to a transform between base_link and aruco_frame
void Mazenavigation::Turtlebot::listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    // declare variables
    geometry_msgs::msg::TransformStamped t_stamped;
    geometry_msgs::msg::Pose pose_out;
    
    // try to get transform between source and target frame
    try
    {
        t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, std::chrono::milliseconds(500));
    }
    
    // print error if transform is not found
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // store the position and orientation of the transform
    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;
    pose_out.orientation = t_stamped.transform.rotation;

    // store the x position of the transform
    x_position_ = pose_out.position.x;
}

// Subscription to AdvancedLogicalCameraImage messages
void Mazenavigation::Turtlebot::turtlebot_part_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg){
   // loop to store the part color and type
   for (const auto& part_pose : msg->part_poses) {
        part_color = part_pose.part.color;
        part_type = part_pose.part.type;
    }
}

// function to listen to a transform between odom and part_frame
void Mazenavigation::Turtlebot::part_listen_transform(const std::string &source_frame, const std::string &target_frame)
{   
    // declare a TransformStamped message and a Pose message
    geometry_msgs::msg::TransformStamped t_stamped_object;
    geometry_msgs::msg::Pose pose_out_object;

    // try to get transform between source and target frame
    try
    {
        t_stamped_object = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, std::chrono::milliseconds(2000));
    }
    // print error if transform is not found    
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Could not get transform between " << source_frame << " and " << target_frame << ": " << ex.what());
        return;
    }

    // store the position and orientation of the part transform
    pose_out_object.position.x = t_stamped_object.transform.translation.x;
    pose_out_object.position.y = t_stamped_object.transform.translation.y;
    pose_out_object.position.z = t_stamped_object.transform.translation.z;
    pose_out_object.orientation = t_stamped_object.transform.rotation;

    // Declare variables for roll, pitch, and yaw
    double roll, pitch, yaw;

    // Convert the quaternion to roll, pitch, and yaw
    tf2::Quaternion quaternion(
        pose_out_object.orientation.x,
        pose_out_object.orientation.y,
        pose_out_object.orientation.z,
        pose_out_object.orientation.w);
    tf2::Matrix3x3 matrix(quaternion);
    matrix.getRPY(roll, pitch, yaw);
   

    // store the first instance of the position and orientation of the part type and color

      if (part_color == mage_msgs::msg::Part::BLUE && part_type == mage_msgs::msg::Part::BATTERY && check_1_ == 0) {
          x_part_pose_.push_back(pose_out_object.position.x);
          y_part_pose_.push_back(pose_out_object.position.y);
          z_part_pose_.push_back(pose_out_object.position.z);
          w_part_pose_.push_back(pose_out_object.orientation.w);
          part_roll_.push_back(roll);
          part_pitch_.push_back(pitch);
          part_yaw_.push_back(yaw);
          part_color_name_.push_back("BLUE");
          check_1_++;
      }
      else if (part_color == mage_msgs::msg::Part::GREEN && part_type == mage_msgs::msg::Part::BATTERY && check_2_ == 0) {
          x_part_pose_.push_back(pose_out_object.position.x);
          y_part_pose_.push_back(pose_out_object.position.y);
          z_part_pose_.push_back(pose_out_object.position.z);
          w_part_pose_.push_back(pose_out_object.orientation.w);
          part_roll_.push_back(roll);
          part_pitch_.push_back(pitch);
          part_yaw_.push_back(yaw);
          part_color_name_.push_back("GREEN");
          check_2_++;
      }
      else if (part_color == mage_msgs::msg::Part::ORANGE && part_type == mage_msgs::msg::Part::BATTERY && check_3_ == 0) {
          x_part_pose_.push_back(pose_out_object.position.x);
          y_part_pose_.push_back(pose_out_object.position.y);
          z_part_pose_.push_back(pose_out_object.position.z);
          w_part_pose_.push_back(pose_out_object.orientation.w);
          part_roll_.push_back(roll);
          part_pitch_.push_back(pitch);
          part_yaw_.push_back(yaw);
          part_color_name_.push_back("ORANGE");
          check_3_++;
      }
}

// function to listen the transformation between odom and part frame
void Mazenavigation::Turtlebot::part_listen_timer_cb()
{
    part_listen_transform("odom", "part_frame");
}

// function to listen the transformation between base_link and aruco_frame
void Mazenavigation::Turtlebot::listen_timer_cb()
{
    listen_transform("base_link", "aruco_frame");
}

// fucntion to store the marker id of the ArucoMarkers
void Mazenavigation::Turtlebot::turtlebot_aruco_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg){
    auto marker_id = msg->marker_ids[0];
    marker_value_=marker_id;
 }

// function to navigate the turtlebot in the maze as per the ArucoMarkers
void Mazenavigation::Turtlebot::turtlebot_navigation_timer_cb() {
  geometry_msgs::msg::Twist bot_velocity_;
  std::string aruco_marker_param_;
  publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  bot_velocity_.linear.x = 0.1;
  publisher_->publish(bot_velocity_);

  // Get the values of the parameters
  if (marker_value_ == 0){
    aruco_marker_param_ = this->get_parameter("aruco_marker_0").as_string();
  }
  else if (marker_value_ == 1){
    aruco_marker_param_ = this->get_parameter("aruco_marker_1").as_string();
  }
  else if (marker_value_ == 2){
     aruco_marker_param_ = this->get_parameter("aruco_marker_2").as_string();
  }

  
  if (x_position_ <= 0.9 && x_position_>= 0.5) {
    // turn right
    if (aruco_marker_param_ == "right_90") {
      bot_velocity_.linear.x = 0.0;
      bot_velocity_.angular.z = -0.1;
      auto start_time = this->now();
      while ((this->now() - start_time).seconds() < (3.14/0.4)){
          publisher_->publish(bot_velocity_);
      }
    }
    // turn left
    else if (aruco_marker_param_ == "left_90"  && x_position_ <= 0.8 && x_position_ >= 0.5) {
      bot_velocity_.linear.x = 0.0;
      bot_velocity_.angular.z = 0.1;
      auto start_time = this->now();
      while ((this->now() - start_time).seconds() < (3.14/0.4)){
          publisher_->publish(bot_velocity_);
      }
    }
    // stop
    else if (aruco_marker_param_ == "end") {
      bot_velocity_.linear.x = 0.0;
      bot_velocity_.angular.z = 0.0;
      publisher_->publish(bot_velocity_);

      // loop to print the position of the parts detected
      while (!print_++) {
        for (int i{0}; i<3; ++i) {
          if (part_color_name_.at(i) == "BLUE") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Blue Battery detected at xyz=[" << x_part_pose_.at(i) << ", " << y_part_pose_.at(i) << ", " << z_part_pose_.at(i) << "] rpy=[" << part_roll_.at(i) << ", " << part_pitch_.at(i) << ", " << part_yaw_.at(i) << "]");
          }

          if (part_color_name_.at(i) == "GREEN") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Green Battery detected at xyz=[" << x_part_pose_.at(i) << ", " << y_part_pose_.at(i) << ", " << z_part_pose_.at(i) << "] rpy=[" << part_roll_.at(i) << ", " << part_pitch_.at(i) << ", " << part_yaw_.at(i) << "]");
          }

          if (part_color_name_.at(i) == "ORANGE") {
            RCLCPP_INFO_STREAM(this->get_logger(), "Orange Battery detected at xyz=[" << x_part_pose_.at(i) << ", " << y_part_pose_.at(i) << ", " << z_part_pose_.at(i) << "] rpy=[" << part_roll_.at(i) << ", " << part_pitch_.at(i) << ", " << part_yaw_.at(i) << "]");
          }
        }   
      }      
    }
  }  
}

// main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Mazenavigation::Turtlebot>("Turtlebot_navigation");
  rclcpp::spin(node);
  rclcpp::shutdown();
}



