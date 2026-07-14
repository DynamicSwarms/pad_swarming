#pragma once
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include "tf2_ros/buffer.hpp"
#include "tf2_ros/qos.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class SmartPadTF
{
public:
    SmartPadTF(
        std::string pad_name,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        rclcpp::Logger parent_logger)
    : m_pad_name(pad_name)
    , m_logger(parent_logger.get_child("SmartPadTF"))
    , m_tf_buffer(std::make_unique<tf2_ros::Buffer>(node_clock_interface->get_clock()))
    {
        RCLCPP_DEBUG(m_logger, "SmartPadTF has been initialized.");

        m_callback_group = node_base_interface->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = m_callback_group;

        m_tf_subscription = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
            node_topics_interface,
            "/tf",
            tf2_ros::DynamicListenerQoS(),
            [this](const std::shared_ptr<const tf2_msgs::msg::TFMessage> msg) {
                m_tf_subscription_callback(std::const_pointer_cast<tf2_msgs::msg::TFMessage>(msg), false);
            },
            sub_opt);
        m_static_tf_subscription = rclcpp::create_subscription<tf2_msgs::msg::TFMessage>(
            node_topics_interface,
            "/tf_static",
            tf2_ros::StaticListenerQoS(),
            [this](const std::shared_ptr<const tf2_msgs::msg::TFMessage> msg) {
                m_tf_subscription_callback(std::const_pointer_cast<tf2_msgs::msg::TFMessage>(msg), true);
            },
            sub_opt);
    }   

    void 
    m_tf_subscription_callback(
        const tf2_msgs::msg::TFMessage::SharedPtr msg,
        bool is_static)
    {
        for (const auto & transform : msg->transforms) {
            m_tf_buffer->setTransform(transform, "default_authority", is_static);
        }
    }

    bool
    get_world_affine3d(Eigen::Affine3d & affine) const { 
        return get_world_affine3d_of(affine, m_pad_name);
    }
    bool 
    get_world_affine3d_of(
        Eigen::Affine3d & affine,
        const std::string & frame) const
    {
        return get_affine3d_in_frame_off(affine, m_world_frame, frame);
    }

    bool 
    get_affine3d_in_frame_off(
        Eigen::Affine3d & affine,
        const std::string & in_frame, 
        const std::string & frame) const
    {        
        geometry_msgs::msg::TransformStamped transform;
        if (lookup_transform(in_frame, frame, transform)) {
            Eigen::Affine3d src = Eigen::Affine3d::Identity();
            tf2::doTransform(src, affine, transform);
            return true;
        }
        return false;
    }

    

    std::vector<std::string>
    get_other_smart_pads() const {
        std::vector<std::string> other_pads;
        std::vector<std::string> frames = m_tf_buffer->getAllFrameNames();
        for (const auto & frame : frames) {
        if (frame.find("smart_pad_") != std::string::npos && frame != m_pad_name) {
                other_pads.push_back(frame);
            }
        }
        return other_pads;
    }


    bool 
    lookup_transform(
        const std::string & target_frame,
        const std::string & source_frame,
        geometry_msgs::msg::TransformStamped & transform) const
    {
        try {
            transform = m_tf_buffer->lookupTransform(target_frame, source_frame, rclcpp::Time(0));
            return true;
        } catch (const tf2::LookupException & ex) {
            RCLCPP_ERROR(m_logger, "LookupException: %s", ex.what());
        } catch (const tf2::ConnectivityException & ex) {
            RCLCPP_ERROR(m_logger, "ConnectivityException: %s", ex.what());
        } catch (const tf2::ExtrapolationException & ex) {
            RCLCPP_ERROR(m_logger, "ExtrapolationException: %s", ex.what());
        } catch (const tf2::InvalidArgumentException & ex) {
            RCLCPP_ERROR(m_logger, "InvalidArgumentException: %s", ex.what());
        }

        return false;
    }

private: 
    std::string m_pad_name;
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>> m_tf_subscription;
    std::shared_ptr<rclcpp::Subscription<tf2_msgs::msg::TFMessage>> m_static_tf_subscription;

    std::string m_world_frame = "world";

};