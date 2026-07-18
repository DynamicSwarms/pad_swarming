#pragma once

#include "padflie_behaviors/pad_client.hpp"
#include "padflies_cpp/commander/padflie_tf.hpp"

class PadClientFactory
{
public:
    PadClientFactory(
        const std::string & prefix,
        std::shared_ptr<PadflieTF> padflie_tf,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::CallbackGroup> callback_group,
        rclcpp::Logger parent_logger)
        : m_prefix(prefix)
        , m_padflie_tf(padflie_tf)
        , m_node_base_interface(node_base_interface)
        , m_node_graph_interface(node_graph_interface)
        , m_node_logging_interface(node_logging_interface)
        , m_node_waitables_interface(node_waitables_interface)
        , m_node_services_interface(node_services_interface)
        , m_callback_group(callback_group)
        , m_logger(parent_logger)
    {
    }

    std::shared_ptr<PadClient> create_pad_client(
        const std::string & pad_name,
        const std::string & pad_right_control_action_name,
        const std::string & pad_idle_target_service_name)
    {
        return std::make_shared<PadClient>(
            m_prefix,
            pad_name,
            pad_right_control_action_name,
            pad_idle_target_service_name,
            m_padflie_tf,
            m_node_base_interface,
            m_node_graph_interface,
            m_node_logging_interface,
            m_node_waitables_interface,
            m_node_services_interface,
            m_callback_group,
            m_logger);
    }

private: 
    std::string m_prefix;    
    std::shared_ptr<PadflieTF> m_padflie_tf;
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_node_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> m_node_graph_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_node_logging_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> m_node_waitables_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> m_node_services_interface;
    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
    rclcpp::Logger m_logger;
  };
