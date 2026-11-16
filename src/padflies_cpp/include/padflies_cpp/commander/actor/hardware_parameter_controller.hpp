#pragma once

#include "rclcpp/rclcpp.hpp"

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

#include <vector>

class HardwareParameterController {
public:
    HardwareParameterController(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        const std::string & cf_prefix,
        rclcpp::Logger parent_logger
    );



    bool get_parameter(const std::string & name, rcl_interfaces::msg::ParameterValue & param_value) const;
    bool get_firmware_parameter(
        const std::string & name,
        rcl_interfaces::msg::ParameterValue & param_value) const;
    void set_parameters(const std::vector<rcl_interfaces::msg::Parameter> & params);

private:
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::GetParameters>> m_get_parameter_client;
    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::GetParameters>> m_get_firmware_parameter_client;
    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> m_set_parameters_client;
};
