#pragma once

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/qos.hpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include "crazyflie_interfaces/msg/pose_named_array.hpp"
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
        const std::string & world_frame,
        std::shared_ptr<rclcpp::Clock> clock,
        rclcpp::Logger logger);

    ~PadflieTF();
    
    void start_listening(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface);

    void m_tf_subscription_callback(
        const std::shared_ptr<tf2_msgs::msg::TFMessage> msg, 
        bool is_static);

    void set_pad(
        const std::string & pad_name);

    bool get_world_affine3d(
        const std::string & frame_id,
        Eigen::Affine3d & affine);
    
    bool get_affine3d_transform(
        const std::string & target_frame,
        const std::string & source_frame,
        Eigen::Affine3d & affine);

    bool affine3d_transform(
        const Eigen::Affine3d & src,
        const std::string & source_frame,
        const std::string & target_frame,
        Eigen::Affine3d & dst);

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

    bool get_cf_pose(
        Eigen::Affine3d & pose);

    bool pose_stamped_to_world_position_and_yaw(
        const geometry_msgs::msg::PoseStamped & pose_stamped,
        Eigen::Vector3d & position,
        double & yaw);

    bool transform_pose_stamped(
        const geometry_msgs::msg::PoseStamped & pose,
        const std::string & target_frame,
        geometry_msgs::msg::PoseStamped & transformed_pose);


    bool can_transform_world(const std::string & source_frame);
private: 

    bool transform_point_stamped(
        const geometry_msgs::msg::PointStamped & point,
        const std::string & target_frame,
        geometry_msgs::msg::PointStamped & transformed_point);

  
    bool lookup_transform(
        const std::string & target_frame,
        const std::string & source_frame,
        geometry_msgs::msg::TransformStamped & transform);

    void cf_positions_callback(
        const crazyflie_interfaces::msg::PoseNamedArray::SharedPtr msg);

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
    std::shared_ptr<rclcpp::Clock> m_clock;
    rclcpp::Logger m_logger;

    std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>> m_tf_subscription;
    std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>> m_static_tf_subscription;


    rclcpp::Subscription<crazyflie_interfaces::msg::PoseNamedArray>::SharedPtr m_cf_positions_sub;
    rclcpp::CallbackGroup::SharedPtr m_callback_group;

private:
    std::string m_pad_name;
};