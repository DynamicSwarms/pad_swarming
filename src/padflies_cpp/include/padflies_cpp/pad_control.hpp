#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "pad_management_interfaces/srv/pad_right_acquire.hpp"
#include "pad_management_interfaces/srv/pad_right_release.hpp"

#include "pad_management_interfaces/srv/pad_idle_target.hpp"

#include <Eigen/Dense>


class PadControl
{
public:
    using RightCallbackT = std::function<void(bool)>;
    

    PadControl(
        const std::string &  prefix, 
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface
    );

    void create_connection(const std::string & pad_name);
    void destroy_connection(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

    bool acquire_right(double timeout_seconds);
    bool release_right();

    void acquire_right_async(double timeout_seconds, RightCallbackT && callback);
    void release_right_async(RightCallbackT && callback);

    bool get_pad_circle_target(
        double timeout_seconds,
        const geometry_msgs::msg::PoseStamped & position, 
        geometry_msgs::msg::PoseStamped & target_position);

private: 
    std::string m_prefix;

    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_node_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> m_node_graph_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> m_node_services_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> m_node_waitables_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_node_logging_interface;
    rclcpp::Logger m_logger;


    rclcpp::Client<pad_management_interfaces::srv::PadRightAcquire>::SharedPtr m_acquire_client;
    rclcpp::Client<pad_management_interfaces::srv::PadRightRelease>::SharedPtr m_release_client;
    rclcpp::Client<pad_management_interfaces::srv::PadIdleTarget>::SharedPtr m_pad_idle_target_client;
};