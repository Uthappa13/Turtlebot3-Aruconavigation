/**
 * @file navigate.h
 * @brief Header file for the Turtlebot Movement
 * 
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include "mage_msgs/msg/part.hpp"

// namespace Mazenavigation 
namespace Mazenavigation {

/**
 * @class Turtlebot
 * @brief A class that represents a Turtlebot node in ROS2
 */
class Turtlebot : public rclcpp::Node {
 /**
   * @brief Construct a new Turtlebot object
   * 
   * @param node_name Name of the node
   */
 public:
  Turtlebot(std::string node_name) : Node(node_name) {
    // Create a timer for navigation
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&Turtlebot::turtlebot_navigation_timer_cb, this));

    // Subscription to the "aruco_markers" topic
    Arucosubscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, std::bind(&Turtlebot::turtlebot_aruco_sub_cb, this, std::placeholders::_1));

    // Declare parameters for Aruco markers
    this->declare_parameter("aruco_marker_0", "default_1"); 
    this->declare_parameter("aruco_marker_1", "default_2");
    this->declare_parameter("aruco_marker_2", "default_3");
    
      // load a buffer of transforms
       tf_buffer_ =
          std::make_unique<tf2_ros::Buffer>(this->get_clock());
       transform_listener_ =
          std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // timer to listen to the transforms
       listen_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&Turtlebot::listen_timer_cb, this));

       // // timer to listen to the transforms
        part_listen_timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&Turtlebot::part_listen_timer_cb, this));
        auto qos_profile = rclcpp::SensorDataQoS();

        // // Create a subscription to advanced logical camera
        partsubscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/advanced_logical_camera/image", qos_profile, std::bind(&Turtlebot::turtlebot_part_sub_cb, this, std::placeholders::_1));

  }

  private:

   /**
    * @brief Publisher attribute for Twist messages
    * 
    */
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
   
   /**
    * @brief Timer Callback function for turtlebot navigation
    * 
    */
   void turtlebot_navigation_timer_cb();

   
   /**
    * @brief Timer attribute
    */
   rclcpp::TimerBase::SharedPtr timer_;


    /**
     * @brief Subscriber attribute to the "aruco_markers" topic
     */    
   rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr Arucosubscription_;
    /**
     * @brief Callback function to subscribe to the "aruco_markers" topic
     */    
   void turtlebot_aruco_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);


    /**
     * @brief Buffer to store transform data for easy lookup
     */
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    
    /**
     * @brief Transform listener to listen for transformation updates
     */
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    
    /**
     * @brief Timer to control the frequency of listening for transforms
     */
    rclcpp::TimerBase::SharedPtr listen_timer_;


    /**
     * @brief Listen to a transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief Timer to listen to the transform
     *
     */
    void listen_timer_cb();


    /**
     * @brief Subscription to mage_msgs::msg::AdvancedLogicalCameraImage messages
     */    
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr partsubscription_;

    /**
     * @brief Timer callback function for the AdvancedLogicalCameraImage topic
     *
     */
    void turtlebot_part_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /**
     * @brief Wall timer object
     */
    rclcpp::TimerBase::SharedPtr part_listen_timer_;

    /**
     * @brief Listen to a transform
     *
     * @param source_frame Source frame (child frame) of the transform
     * @param target_frame Target frame (parent frame) of the transform
     */
    void part_listen_transform(const std::string &source_frame, const std::string &target_frame);

    /**
     * @brief Timer to listen to the transform
     *
     */
    void part_listen_timer_cb();


    // declare variables for battery type and color
    int part_color{};

    int part_type{};

    // declaration of counter variables
    int check_1_{0};

    int check_2_{0};

    int check_3_{0};

    int print_{0};

    // declaration of vectors to store pose
    std::vector<double> x_part_pose_;

    std::vector<double> y_part_pose_;

    std::vector<double> z_part_pose_;

    std::vector<double> w_part_pose_;

    std::vector<double> part_roll_;

    std::vector<double> part_pitch_;

    std::vector<double> part_yaw_;

    // declaration of vectors to handle part color and type
    std::vector<std::string> part_color_name_;

	std::vector<int>part_color_;

    std::vector<int>part_type_;

    // declaration of aruco marker parameter variables
    std::string aruco_marker_0_;

    std::string aruco_marker_1_;

    std::string aruco_marker_2_;

    // declaration of variable to store aruco marker value
    int marker_value_{};

    // declaration of variable to store aruco marker position
    double x_position_{};



};  // class Turtlebot
}  // namespace Mazenavigation