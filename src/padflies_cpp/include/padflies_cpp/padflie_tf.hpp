#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "crazyflie_interfaces/msg/pose_stamped_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <Eigen/Dense>
#include <optional>

class PadflieTF
{
public:
    using Duration = rclcpp::Duration;
    using Time = rclcpp::Time;

    PadflieTF(
        const std::string & cf_name,
        const std::string & world_frame = "world");

    ~PadflieTF();
    
    void on_configure(
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

    void set_pad(
        const std::string & pad_name);
    
        const std::string & get_pad_name() const { return m_pad_name; }

    bool can_transform(const std::string & frame);

    /**
     * Get the position of the pad with a timeout.
     * If the pad is not found within the timeout it will raise an exception.
     */
    bool get_pad_position_and_yaw_or_timeout(
        rclcpp::Duration & timeout_sec,
        Eigen::Vector3d & position,
        double & yaw);

    /**
     * Tries to get the position of the pad.
     */
    bool get_pad_position_and_yaw(
        Eigen::Vector3d & position,
        double & yaw);

    /**
     * Returns a pose of the pad (in pad_frame) as a TransformStamped.
     */
    bool get_pad_pose(
        geometry_msgs::msg::PoseStamped & pose_stamped);
    
    /**
     * Tries to get the pose of the pad in the world frame.
     */
    bool get_pad_pose_world(
        geometry_msgs::msg::PoseStamped & pose_stamped);
  

    /**
     * Returns the position of the cf in a given frame.
     */
    bool get_cf_pose_stamped(
        const std::string & frame_id,
        geometry_msgs::msg::PoseStamped & pose_stamped);
    
    /**
     * Returns the position of the cf in world frame, as a vector of doubles.
     */
    bool get_cf_position(
        Eigen::Vector3d & position);

    bool pose_stamped_to_world_position_and_yaw(
        const geometry_msgs::msg::PoseStamped & pose_stamped,
        Eigen::Vector3d & position,
        double & yaw);
private: 

    bool transform_point_stamped(
        const geometry_msgs::msg::PointStamped & point,
        const std::string & target_frame,
        geometry_msgs::msg::PointStamped & transformed_point);

    bool transform_pose_stamped(
        const geometry_msgs::msg::PoseStamped & pose,
        const std::string & target_frame,
        geometry_msgs::msg::PoseStamped & transformed_pose);

    bool lookup_transform(
        const std::string & target_frame,
        const std::string & source_frame,
        geometry_msgs::msg::TransformStamped & transform);

    void cf_positions_callback(
        const crazyflie_interfaces::msg::PoseStampedArray::SharedPtr msg);

private: 
    template<typename... Args>
    void log(const char* format, Args&&... args);

    rclcpp::Time get_now();


private:
    std::string m_cf_name;
    std::string m_world_frame;

    bool m_has_pad;


    geometry_msgs::msg::PoseStamped m_last_position;
    rclcpp::Time m_last_position_time;
    rclcpp::Duration m_position_timeout;
    
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

private:

    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> m_node;

    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;

    rclcpp::Subscription<crazyflie_interfaces::msg::PoseStampedArray>::SharedPtr m_cf_positions_sub;
    rclcpp::CallbackGroup::SharedPtr m_callback_group;

private:
    std::string m_pad_name;
    std::string m_logger_name;
};