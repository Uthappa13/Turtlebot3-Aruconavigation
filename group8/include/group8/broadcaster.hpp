/**
 * @file broadcaster.hpp
 * @brief Header file for the Broadcaster class
 */
#pragma once


#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <limits>


using namespace std::chrono_literals;

/**
 * @class Broadcaster
 * @brief node to broadcast transforms for Aruco markers and batteries
 */
class Broadcaster : public rclcpp::Node
{
public:
    Broadcaster(std::string node_name) : Node(node_name)
    {
        // parameter to decide whether to execute the broadcaster or not
        RCLCPP_INFO(this->get_logger(), "Broadcaster started");

        // initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);

        // timer to publish the transform
        aruco_broadcast_timer_ = this->create_wall_timer(
            1s,
            std::bind(&Broadcaster::aruco_broadcast_timer_cb, this));

        // timer to publish the transform
        battery_broadcast_timer_ = this->create_wall_timer(
            2s,
            std::bind(&Broadcaster::battery_broadcast_timer_cb, this));

        // Define the Quality of Service profile for the subscriptions
        auto qos_profile = rclcpp::SensorDataQoS();

        // Create a subscription to the "aruco_markers" topic
        Arucosubscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10, std::bind(&Broadcaster::turtlebot_aruco_sub_cb, this, std::placeholders::_1));

        // Create a subscription to the "/mage/advanced_logical_camera/image" topic
        Batterysubscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/advanced_logical_camera/image", qos_profile, std::bind(&Broadcaster::turtlebot_battery_sub_cb, this, std::placeholders::_1));

        // Create a subscription to the clock topic
        clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&Broadcaster::sub_clock_cb, this, std::placeholders::_1));
    }


private:
    
    /**
     * @brief Storing transforms
     */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    
    /**
     * @brief Transform listener
     */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    /**
     * @brief Timer to periodically publish the transform for Aruco markers
     */
    rclcpp::TimerBase::SharedPtr aruco_broadcast_timer_;
    
    /**
     * @brief Timer to periodically publish the transform for the battery
     */
    rclcpp::TimerBase::SharedPtr battery_broadcast_timer_;

    /**
     * @brief TransformStamped message for Aruco dynamic transform
     */
    geometry_msgs::msg::TransformStamped aruco_dynamic_transform_stamped_;

    /**
     * @brief TransformStamped message for battery dynamic transform
     */
    geometry_msgs::msg::TransformStamped battery_dynamic_transform_stamped_;

    /**
     * @brief Current Time
     */
    rclcpp::Time current_time_;

    /**
     * @brief Callback function for the Aruco broadcast timer
     *
     */
    void aruco_broadcast_timer_cb();

    /**
     * @brief Callback function for the battery broadcast timer
     *
     */
    void battery_broadcast_timer_cb();

    /**
     * @brief Subscription to clock topic
     */
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_subscription_;

    /**
     * @brief Callback function for the clock subscription
     *
     * @param msg Message received from the clock topic
     */
    void sub_clock_cb(const rosgraph_msgs::msg::Clock::SharedPtr msg);

    /**
     * @brief Subscription to the "aruco_markers" topic
     */    
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr Arucosubscription_;
    
    /**
     * @brief Callback function for the aruco marker broadcast
     *
     * @param msg Message received from the "aruco_markers" topic
     */
    void turtlebot_aruco_sub_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

    /**
     * @brief Subscription to the "/mage/advanced_logical_camera/image" topic
     */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr Batterysubscription_;
    
    /**
     * @brief Callback function for the battery position broadcast
     *
     * @param msg Message received from the "/mage/advanced_logical_camera/image" topic
     */
    void turtlebot_battery_sub_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
};


