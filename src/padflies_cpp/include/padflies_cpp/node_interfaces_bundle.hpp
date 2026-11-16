#pragma once

#include "rclcpp/rclcpp.hpp"

namespace padflies_cpp
{
    struct NodeInterfacesBundle
    {
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> services_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> graph_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> waitables_interface;
    };
} // namespace padflies_cpp