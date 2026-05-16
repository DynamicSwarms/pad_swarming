#pragma once

#include "padflie_lifecycle_connection.hpp"
#include "padflie_control_connection.hpp"

namespace rqt_padflies
{
    class PadflieROSConnection
    {
    public:
        PadflieROSConnection(
            std::string prefix,
            std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface)
        : m_logger(node_logging_interface->get_logger())
        , m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
        , m_lifecycle_connection(std::make_shared<PadflieLifecycleConnection>(
                prefix,
                node_topics_interface,
                node_base_interface,
                node_graph_interface,
                node_services_interface,
                m_callback_group,
                m_logger))
        , m_control_connection(std::make_shared<PadflieControlConnection>(
                prefix,
                node_topics_interface,
                node_base_interface,
                node_graph_interface,
                node_services_interface,
                m_callback_group
            ))
        {
            m_lifecycle_state_polling_timer = rclcpp::create_timer(
                node_base_interface,
                node_timers_interface,
                std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME), 
                std::chrono::milliseconds(1000),
                std::bind(&PadflieROSConnection::poll_current_lifecycle_state, this)
            );


        }

        void log_info(const std::string &message) {
            RCLCPP_INFO(m_logger, "%s", message.c_str());
        }

        std::shared_ptr<PadflieLifecycleConnection> get_lifecycle_connection() {
            return m_lifecycle_connection;
        }

        std::shared_ptr<PadflieControlConnection> get_control_connection() {
            return m_control_connection;
        }

private:
    void poll_current_lifecycle_state() {
        m_lifecycle_connection->poll_current_lifecycle_state();
    }


private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
    std::shared_ptr<PadflieLifecycleConnection> m_lifecycle_connection;
    std::shared_ptr<PadflieControlConnection> m_control_connection;

    std::shared_ptr<rclcpp::TimerBase> m_lifecycle_state_polling_timer;
};

} // namespace rqt_padflies