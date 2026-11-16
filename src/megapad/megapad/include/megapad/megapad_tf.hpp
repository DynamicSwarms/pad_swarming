#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pad_management_cpp/I_pad_resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/qos.hpp"

namespace megapad
{

class MegaPadTF
{
public:
    explicit MegaPadTF(
        pad_management_cpp::NodeInterfacesBundle interfaces,
        rclcpp::Logger parent_logger)
    : m_logger(parent_logger.get_child("MegaPadTF"))
    , m_tf_buffer(std::make_unique<tf2_ros::Buffer>(
        interfaces.clock_interface->get_clock()))
    {
        m_callback_group = interfaces.base_interface->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions options;
        options.callback_group = m_callback_group;

        m_tf_subscription = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
            interfaces.topics_interface, "/tf", tf2_ros::DynamicListenerQoS(),
            [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
                update_buffer(*msg, false);
            },
            options);
        m_static_tf_subscription = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
            interfaces.topics_interface, "/tf_static", tf2_ros::StaticListenerQoS(),
            [this](const tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
                update_buffer(*msg, true);
            },
            options);
    }

    std::vector<std::string> get_frame_names() const
    {
        return m_tf_buffer->getAllFrameNames();
    }

    bool lookup_transform(
        const std::string & target_frame,
        const std::string & source_frame,
        geometry_msgs::msg::TransformStamped & transform) const
    {
        try {
            transform = m_tf_buffer->lookupTransform(
                target_frame, source_frame, rclcpp::Time(0));
            return true;
        } catch (const tf2::TransformException & exception) {
            RCLCPP_DEBUG(
                m_logger, "Cannot transform %s to %s: %s",
                source_frame.c_str(), target_frame.c_str(), exception.what());
            return false;
        }
    }

    bool get_frame_pose(
        const std::string & target_frame,
        const std::string & source_frame,
        geometry_msgs::msg::PoseStamped & pose) const
    {
        geometry_msgs::msg::TransformStamped transform;
        if (!lookup_transform(target_frame, source_frame, transform)) {
            return false;
        }

        pose.header = transform.header;
        pose.pose.position.x = transform.transform.translation.x;
        pose.pose.position.y = transform.transform.translation.y;
        pose.pose.position.z = transform.transform.translation.z;
        pose.pose.orientation = transform.transform.rotation;
        return true;
    }

    bool get_world_pose(
        const std::string & source_frame,
        geometry_msgs::msg::PoseStamped & pose) const
    {
        return get_frame_pose("world", source_frame, pose);
    }

    bool get_world_position_2d(
        const geometry_msgs::msg::PoseStamped & source,
        Eigen::Vector2d & position) const
    {
        geometry_msgs::msg::TransformStamped transform;
        if (source.header.frame_id.empty() ||
            !lookup_transform("world", source.header.frame_id, transform))
        {
            return false;
        }

        const auto & rotation = transform.transform.rotation;
        const Eigen::Quaterniond quaternion(
            rotation.w, rotation.x, rotation.y, rotation.z);
        const Eigen::Vector3d source_position(
            source.pose.position.x,
            source.pose.position.y,
            source.pose.position.z);
        const Eigen::Vector3d translation(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z);
        const Eigen::Vector3d world_position = quaternion * source_position + translation;
        position = world_position.head<2>();
        return true;
    }

private:
    void update_buffer(const tf2_msgs::msg::TFMessage & message, bool is_static)
    {
        for (const auto & transform : message.transforms) {
            m_tf_buffer->setTransform(transform, "megapad", is_static);
        }
    }

    rclcpp::Logger m_logger;
    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_tf_subscription;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr m_static_tf_subscription;
};

}  // namespace megapad
