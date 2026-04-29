#include "padflies_cpp/pad_control.hpp"

static std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> m_callback_groups;
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

PadControl::PadControl(
    const std::string & prefix,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface)
: m_prefix(prefix)
, m_node_base_interface(node_base_interface)
, m_node_graph_interface(node_graph_interface)
, m_node_services_interface(node_services_interface)
, m_node_waitables_interface(node_waitables_interface)
, m_node_logging_interface(node_logging_interface)
, m_logger(node_logging_interface->get_logger())
{
    if (m_callback_groups.find(prefix) == m_callback_groups.end())
        m_callback_groups[prefix] = node_base_interface->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
}

void PadControl::create_connection(const std::string & pad_name)
{
    
    m_acquire_client = rclcpp::create_client<pad_management_interfaces::srv::PadRightAcquire>(
        m_node_base_interface, 
        m_node_graph_interface,
        m_node_services_interface,
        "/" + pad_name + "/pad_right_acquire",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[m_prefix]);
    m_release_client = rclcpp::create_client<pad_management_interfaces::srv::PadRightRelease>(
        m_node_base_interface,
        m_node_graph_interface,
        m_node_services_interface,
        "/" + pad_name + "/pad_right_release",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[m_prefix]);
    m_pad_idle_target_client = rclcpp::create_client<pad_management_interfaces::srv::PadIdleTarget>(
        m_node_base_interface,
        m_node_graph_interface,
        m_node_services_interface,
        "/" + pad_name + "/pad_idle_target",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[m_prefix]);
}

void PadControl::destroy_connection(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    m_acquire_client.reset();
    m_release_client.reset();
    m_pad_idle_target_client.reset();
}


bool PadControl::acquire_right(double timeout_seconds)
{
    if (!m_acquire_client) return false;

    if (!m_acquire_client->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_seconds)))) {
        RCLCPP_ERROR(m_logger, "Service not available for acquiring pad right");
        return false;
    }

    auto request = std::make_shared<pad_management_interfaces::srv::PadRightAcquire::Request>();
    request->name = m_prefix;
    request->timeout = timeout_seconds;

    auto result = m_acquire_client->async_send_request(request);
    auto status = result.wait_for(std::chrono::seconds(static_cast<int>(timeout_seconds)));
    if (status == std::future_status::ready)
    {
        auto response = result.get();
        return response->success;
    } 

    return false;
}


bool PadControl::release_right()
{
    if (!m_release_client) return false;

    if (!m_release_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(m_logger, "Service not available for releasing pad right");
        return false;
    }

    auto request = std::make_shared<pad_management_interfaces::srv::PadRightRelease::Request>();
    request->name = m_prefix;

    auto result = m_release_client->async_send_request(request);
    auto status = result.wait_for(std::chrono::seconds(1));
    
    if (status == std::future_status::ready)
    {
        auto response = result.get();
        return response->success;
    } 

    return false;
}

void PadControl::acquire_right_async(double timeout_seconds, RightCallbackT && callback)
{
    if (m_acquire_client && m_acquire_client->wait_for_service(std::chrono::milliseconds(100))) {
        auto request = std::make_shared<pad_management_interfaces::srv::PadRightAcquire::Request>();
        request->name = m_prefix;
        request->timeout = timeout_seconds;

        m_acquire_client->async_send_request(
            request, 
            [this, callback](rclcpp::Client<pad_management_interfaces::srv::PadRightAcquire>::SharedFutureWithRequest response_future) {
                auto response = response_future.get().second;
                RCLCPP_INFO(m_logger, "Pad right acquired: %s", response->success ? "true" : "false");
                callback(response->success);
            });
    } else {      
        RCLCPP_ERROR(m_logger, "Service not available for acquiring pad right");
        callback(false);
    }
}

void PadControl::release_right_async(RightCallbackT && callback)
{
    if (m_release_client && m_release_client->wait_for_service(std::chrono::milliseconds(100))) {  
        auto request = std::make_shared<pad_management_interfaces::srv::PadRightRelease::Request>();
        request->name = m_prefix;
        m_release_client->async_send_request(
            request, 
            [this, callback](rclcpp::Client<pad_management_interfaces::srv::PadRightRelease>::SharedFutureWithRequest response_future) {
                auto response = response_future.get().second;
                RCLCPP_INFO(m_logger, "Pad right released: %s", response->success ? "true" : "false");
                callback(response->success);
            });
    }  else {
        RCLCPP_ERROR(m_logger, "Service not available for releasing pad right");
        callback(false);
    }    
}

bool PadControl::get_pad_circle_target(
    double timeout_seconds,
    const geometry_msgs::msg::PoseStamped & position, 
    geometry_msgs::msg::PoseStamped & target_position)
{
    if (!m_pad_idle_target_client) return false;

    if (!m_pad_idle_target_client->wait_for_service(std::chrono::milliseconds((long int)(timeout_seconds * 1000)))) {
        RCLCPP_ERROR(m_logger, "Service not available for getting pad circle target");
        return false;
    }

    auto request = std::make_shared<pad_management_interfaces::srv::PadIdleTarget::Request>();
    request->name = m_prefix;
    request->position = position;

    auto result = m_pad_idle_target_client->async_send_request(request);
    auto status = result.wait_for(std::chrono::milliseconds((long int)(timeout_seconds * 1000)));
    

    if (status == std::future_status::ready)
    {
        auto response = result.get();
        target_position = response->target;
        return true;
    } 

    return false;
}

