#include "padflies_cpp/hardware_parameter_controller.hpp"

static std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> m_callback_groups;
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

HardwareParameterController::HardwareParameterController()
{
}

void
HardwareParameterController::configure(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & cf_prefix
)
{
    if (m_callback_groups.find(cf_prefix) == m_callback_groups.end())
        m_callback_groups[cf_prefix] = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    m_get_parameter_client = node->create_client<rcl_interfaces::srv::GetParameters>(
        cf_prefix+"/get_parameters",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[cf_prefix]);
    m_set_parameters_client = node->create_client<rcl_interfaces::srv::SetParameters>(
        cf_prefix+"/set_parameters",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[cf_prefix]);
}


bool
HardwareParameterController::get_parameter(
    const std::string & name, 
    rcl_interfaces::msg::ParameterValue & param_value) const
{
    if (!m_get_parameter_client) return false;

    if (!m_get_parameter_client->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_WARN(rclcpp::get_logger("HardwareParameterController"), "GetParameters service not available");
        return false;
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names.push_back(name);

    auto result = m_get_parameter_client->async_send_request(request);
    auto status = result.wait_for(std::chrono::milliseconds(100));
    
    if (status == std::future_status::ready)
    {
        auto response = result.get();
        if (!response->values.empty()) {
            param_value = response->values[0];
            return true;
        } else RCLCPP_WARN(rclcpp::get_logger("HardwareParameterController"), "Parameter %s not found", name.c_str());
    }
    
    return false;
}

void
HardwareParameterController::set_parameters(
    const std::vector<rcl_interfaces::msg::Parameter> & params)
{
    if (!m_set_parameters_client) return;

    if (!m_set_parameters_client->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_WARN(rclcpp::get_logger("HardwareParameterController"), "SetParameters service not available");
        return;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (const auto & param : params) {
        request->parameters.push_back(param);
    }

    auto result = m_set_parameters_client->async_send_request(request);
}
