#include "padflies_cpp/commander/actor/hardware_parameter_controller.hpp"

static std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> m_callback_groups;
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

HardwareParameterController::HardwareParameterController(        
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    const std::string & cf_prefix,
    rclcpp::Logger parent_logger)
    : m_logger(parent_logger.get_child("HardwareParameterController"))
{
    if (m_callback_groups.find(cf_prefix) == m_callback_groups.end())
        m_callback_groups[cf_prefix] = node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    m_get_parameter_client = rclcpp::create_client<rcl_interfaces::srv::GetParameters>(
        node_base_interface,
        node_graph_interface,
        node_services_interface,
        cf_prefix+"/get_parameters",
        rclcpp::ServicesQoS(),
        m_callback_groups[cf_prefix]);
    m_set_parameters_client = rclcpp::create_client<rcl_interfaces::srv::SetParameters>(
        node_base_interface,
        node_graph_interface,
        node_services_interface,
        cf_prefix+"/set_parameters",
        rclcpp::ServicesQoS(),
        m_callback_groups[cf_prefix]);
}


bool
HardwareParameterController::get_parameter(
    const std::string & name, 
    rcl_interfaces::msg::ParameterValue & param_value) const
{
    if (!m_get_parameter_client) return false;

    if (!m_get_parameter_client->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_WARN(m_logger, "GetParameters service not available");
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
        } else RCLCPP_DEBUG(m_logger, "Parameter %s not found", name.c_str());
    }
    
    return false;
}

void
HardwareParameterController::set_parameters(
    const std::vector<rcl_interfaces::msg::Parameter> & params)
{
    if (!m_set_parameters_client) return;

    if (!m_set_parameters_client->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_WARN(m_logger, "SetParameters service not available");
        return;
    }

    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    for (const auto & param : params) {
        request->parameters.push_back(param);
    }

    auto result = m_set_parameters_client->async_send_request(request);
}