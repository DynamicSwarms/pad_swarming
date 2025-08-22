#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rcl_interfaces/msg/parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"

#include <vector>

class HardwareParameterController {
public:
    HardwareParameterController();

    void configure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, const std::string & cf_prefix);


    bool get_parameter(const std::string & name, rcl_interfaces::msg::ParameterValue & param_value) const;
    void set_parameters(const std::vector<rcl_interfaces::msg::Parameter> & params);

private:

    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::GetParameters>> m_get_parameter_client;
    std::shared_ptr<rclcpp::Client<rcl_interfaces::srv::SetParameters>> m_set_parameters_client;
};
