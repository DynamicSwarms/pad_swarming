#include "padflies_cpp/pad_control.hpp"

static std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> m_callback_groups;
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

PadControl::PadControl()
: m_logger_name("PadControlNoNode")
{
}

void PadControl::set_node(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    m_node = node;
}

void PadControl::activate(
        const std::string & pad_name,
        const std::string & prefix
)
{
    m_prefix = prefix;

    auto node = m_node.lock();
    if (!node)
    {
        RCLCPP_ERROR(rclcpp::get_logger(m_logger_name), "Node is null, cannot activate PadControl, will probably segfault now.");
        return;
    } 
        
    if (m_callback_groups.find(prefix) == m_callback_groups.end())
        m_callback_groups[prefix] = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    m_acquire_client = node->create_client<pad_management_interfaces::srv::PadRightAcquire>(
        "/"+pad_name+"/pad_right_acquire",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[prefix]);
    m_release_client = node->create_client<pad_management_interfaces::srv::PadRightRelease>(
        "/"+pad_name+"/pad_right_release", 
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[prefix]);
    m_pad_idle_target_client = node->create_client<pad_management_interfaces::srv::PadIdleTarget>(
        "/"+pad_name+"/pad_idle_target",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[prefix]);

    m_logger_name = node->get_name();
}

void PadControl::deactivate()
{
    m_acquire_client.reset();
    m_release_client.reset();
    m_pad_idle_target_client.reset();
}

void PadControl::acquire_right_async(double timeout_seconds, RightCallbackT && callback)
{
    RCLCPP_INFO(rclcpp::get_logger(m_logger_name), "Requesting pad right acquisition");
    if (m_acquire_client && m_acquire_client->wait_for_service(std::chrono::milliseconds(100))) {
        auto request = std::make_shared<pad_management_interfaces::srv::PadRightAcquire::Request>();
        request->name = m_prefix;
        request->timeout = timeout_seconds;

        m_acquire_client->async_send_request(
            request, 
            [this, callback](rclcpp::Client<pad_management_interfaces::srv::PadRightAcquire>::SharedFutureWithRequest response_future) {
                auto response = response_future.get().second;
                RCLCPP_INFO(rclcpp::get_logger(m_logger_name), "Pad right acquired: %s", response->success ? "true" : "false");
                callback(response->success);
            });
    } else {      
        RCLCPP_ERROR(rclcpp::get_logger(m_logger_name), "Service not available for acquiring pad right");
        callback(false);
    }
}

void PadControl::release_right_async(RightCallbackT && callback)
{
    RCLCPP_INFO(rclcpp::get_logger(m_logger_name), "Requesting pad right release");
    if (m_release_client && m_release_client->wait_for_service(std::chrono::milliseconds(100))) {  
        auto request = std::make_shared<pad_management_interfaces::srv::PadRightRelease::Request>();
        request->name = m_prefix;
        m_release_client->async_send_request(
            request, 
            [this, callback](rclcpp::Client<pad_management_interfaces::srv::PadRightRelease>::SharedFutureWithRequest response_future) {
                auto response = response_future.get().second;
                RCLCPP_INFO(rclcpp::get_logger(m_logger_name), "Pad right released: %s", response->success ? "true" : "false");
                callback(response->success);
            });
    }  else {
        RCLCPP_ERROR(rclcpp::get_logger(m_logger_name), "Service not available for releasing pad right");
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
        RCLCPP_ERROR(rclcpp::get_logger(m_logger_name), "Service not available for getting pad circle target");
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

