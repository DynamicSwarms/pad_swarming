#include "padflies_cpp/pad_control.hpp"

PadControl::PadControl()
{
}

void PadControl::configure(const std::string & cf_prefix,
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    m_cf_prefix = cf_prefix;
    m_callback_group = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    m_acquire_client = node->create_client<pad_management_interfaces::srv::PadRightAcquire>(
        "pad_right_acquire",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_group);
    m_release_client = node->create_client<pad_management_interfaces::srv::PadRightRelease>(
        "pad_right_release", 
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_group);
    m_circle_behaviour_client = node->create_client<pad_management_interfaces::srv::PadCircleBehaviour>(
        "pad_circle_behaviour",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_group);
}

void PadControl::unconfigure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    m_acquire_client.reset();
    m_release_client.reset();
    m_circle_behaviour_client.reset();

    m_callback_group.reset();
}


bool PadControl::acquire_right(double timeout_seconds)
{
    if (!m_acquire_client->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_seconds)))) {
        RCLCPP_ERROR(rclcpp::get_logger("PadControl"), "Service not available for acquiring pad right");
        return false;
    }

    auto request = std::make_shared<pad_management_interfaces::srv::PadRightAcquire::Request>();
    request->name = m_cf_prefix;
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
    if (!m_release_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(rclcpp::get_logger("PadControl"), "Service not available for releasing pad right");
        return false;
    }

    auto request = std::make_shared<pad_management_interfaces::srv::PadRightRelease::Request>();
    request->name = m_cf_prefix;

    auto result = m_release_client->async_send_request(request);
    auto status = result.wait_for(std::chrono::seconds(1));
    
    if (status == std::future_status::ready)
    {
        auto response = result.get();
        return response->success;
    } 

    return false;
}

bool PadControl::get_pad_circle_target(
    double timeout_seconds,
    Eigen::Vector3d & position, 
    Eigen::Vector3d & target_position)
{
    if (!m_circle_behaviour_client->wait_for_service(std::chrono::duration<double>(timeout_seconds))) {
        RCLCPP_ERROR(rclcpp::get_logger("PadControl"), "Service not available for getting pad circle target");
        return false;
    }

    auto request = std::make_shared<pad_management_interfaces::srv::PadCircleBehaviour::Request>();
    request->name = m_cf_prefix;
    request->position = geometry_msgs::msg::Point();
    request->position.x = position.x();
    request->position.y = position.y();
    request->position.z = position.z();

    auto result = m_circle_behaviour_client->async_send_request(request);
    auto status = result.wait_for(std::chrono::duration<double>(timeout_seconds));
    
    if (status == std::future_status::ready)
    {
        auto response = result.get();
        target_position.x() = response->target.x;
        target_position.y() = response->target.y;
        target_position.z() = response->target.z;
        return true;
    } 

    return false;
}

