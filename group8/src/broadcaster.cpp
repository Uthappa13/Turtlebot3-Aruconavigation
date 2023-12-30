#include <broadcaster.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>

// allows to use, 50ms, etc
using namespace std::chrono_literals;

// callback function for the timer that broadcasts the Aruco marker's transform
void Broadcaster::aruco_broadcast_timer_cb()
{ 
    aruco_dynamic_transform_stamped_.header.stamp = current_time_;
    aruco_dynamic_transform_stamped_.header.frame_id = "camera_rgb_optical_frame";
    aruco_dynamic_transform_stamped_.child_frame_id = "aruco_frame";
    // send the transform
    tf_broadcaster_->sendTransform(aruco_dynamic_transform_stamped_);
}

// callback function for the subscription to ArucoMarkers messages
void Broadcaster::turtlebot_aruco_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{

  // get the pose of the Aruco marker
  auto pose = msg->poses[0];

  // translation and rotation of the transform
  aruco_dynamic_transform_stamped_.transform.translation.x = pose.position.x;
  aruco_dynamic_transform_stamped_.transform.translation.y = pose.position.y;
  aruco_dynamic_transform_stamped_.transform.translation.z = pose.position.z;

  aruco_dynamic_transform_stamped_.transform.rotation.x = pose.orientation.x;
  aruco_dynamic_transform_stamped_.transform.rotation.y = pose.orientation.y;
  aruco_dynamic_transform_stamped_.transform.rotation.z = pose.orientation.z;
  aruco_dynamic_transform_stamped_.transform.rotation.w = pose.orientation.w;
}

// callback function for the subscription to the clock
void Broadcaster::sub_clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
   // get the current time
   current_time_ = msg->clock;
}

// callback function for the timer that broadcasts the battery's transform
void Broadcaster::battery_broadcast_timer_cb()
{   
    // set the timestamp and frame IDs for the transform
    battery_dynamic_transform_stamped_.header.stamp = current_time_;
    battery_dynamic_transform_stamped_.header.frame_id = "logical_camera_link";
    battery_dynamic_transform_stamped_.child_frame_id = "part_frame";

    // send the transform
    tf_broadcaster_->sendTransform(battery_dynamic_transform_stamped_);
}

// callback function for the subscription to ArucoMarkers messages
void Broadcaster::turtlebot_battery_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

  // loop for each part
  for (const auto& part_pose : msg->part_poses) {
      // get the pose and part
      auto pose = part_pose.pose;

      // translation of the transform
      battery_dynamic_transform_stamped_.transform.translation.x = pose.position.x;
      battery_dynamic_transform_stamped_.transform.translation.y = pose.position.y;
      battery_dynamic_transform_stamped_.transform.translation.z = pose.position.z;

      // rotation of the transform
      battery_dynamic_transform_stamped_.transform.rotation.x = pose.orientation.x;
      battery_dynamic_transform_stamped_.transform.rotation.y = pose.orientation.y;
      battery_dynamic_transform_stamped_.transform.rotation.z = pose.orientation.z;
      battery_dynamic_transform_stamped_.transform.rotation.w = pose.orientation.w;
  }

}

// main function
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Broadcaster>("broadcaster");
  rclcpp::spin(node);
  rclcpp::shutdown();
}

















