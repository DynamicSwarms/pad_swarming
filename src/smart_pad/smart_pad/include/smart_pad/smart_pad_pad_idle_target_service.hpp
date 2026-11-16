#pragma once

#include "rclcpp/rclcpp.hpp"
#include "pad_management_interfaces/srv/pad_idle_target.hpp"
#include "smart_pad/smart_pad_tf.hpp"
class SmartPadPadIdleTargetService
{
public:
    SmartPadPadIdleTargetService(
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
        rclcpp::node_interfaces::NodeServicesInterface::SharedPtr node_services_interface,
        std::shared_ptr<SmartPadTF> smart_pad_tf,
        rclcpp::Logger parent_logger)
    : m_logger(parent_logger)
    , m_smart_pad_tf(smart_pad_tf)
    , m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    {
        m_service = rclcpp::create_service<pad_management_interfaces::srv::PadIdleTarget>(
            node_base_interface,
            node_services_interface,
            "~/pad_idle_target",
            std::bind(&SmartPadPadIdleTargetService::handle_service, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(),
            m_callback_group
        );
    }
    
    void handle_service(
        const std::shared_ptr<pad_management_interfaces::srv::PadIdleTarget::Request> request,
        std::shared_ptr<pad_management_interfaces::srv::PadIdleTarget::Response> response)
    {
        (void)request;
        // For now, we just return the current position of the pad as the idle target
        Eigen::Affine3d current_pose;
        if (!m_smart_pad_tf->get_world_affine3d(current_pose)) {
            RCLCPP_ERROR(m_logger, "Failed to get world pose for pad");
            return;
        }
        response->target.pose.position.x = current_pose.translation().x();
        response->target.pose.position.y = current_pose.translation().y();
        response->target.pose.position.z = current_pose.translation().z() + 1;
        Eigen::Quaterniond q(current_pose.rotation());
        response->target.pose.orientation.x = q.x();
        response->target.pose.orientation.y = q.y();
        response->target.pose.orientation.z = q.z();
        response->target.pose.orientation.w = q.w();
        response->target.header.frame_id = "world";   
    }
    private: 
        rclcpp::Logger m_logger;
        std::shared_ptr<rclcpp::Service<pad_management_interfaces::srv::PadIdleTarget>> m_service;
        std::shared_ptr<SmartPadTF> m_smart_pad_tf;
        rclcpp::CallbackGroup::SharedPtr m_callback_group;
};