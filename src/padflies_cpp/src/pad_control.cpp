#include "padflies_cpp/pad_control.hpp"

static std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> m_callback_groups;
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

PadControl::PadControl()
: m_frame_name("pad_circle")
{
}

void PadControl::configure(const std::string & prefix,
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    m_prefix = prefix;
    if (m_callback_groups.find(prefix) == m_callback_groups.end())
        m_callback_groups[prefix] = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    m_acquire_client = node->create_client<pad_management_interfaces::srv::PadRightAcquire>(
        "acquire_pad_right",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[prefix]);
    m_release_client = node->create_client<pad_management_interfaces::srv::PadRightRelease>(
        "release_pad_right", 
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[prefix]);
    m_circle_behaviour_client = node->create_client<pad_management_interfaces::srv::PadCircleBehaviour>(
        "pad_circle",
        rclcpp::QoS(10).get_rmw_qos_profile(),
        m_callback_groups[prefix]);
}

void PadControl::unconfigure(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    m_acquire_client.reset();
    m_release_client.reset();
    m_circle_behaviour_client.reset();
}


bool PadControl::acquire_right(double timeout_seconds)
{
    if (!m_acquire_client) return false;

    if (!m_acquire_client->wait_for_service(std::chrono::seconds(static_cast<int>(timeout_seconds)))) {
        RCLCPP_ERROR(rclcpp::get_logger("PadControl"), "Service not available for acquiring pad right");
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
        RCLCPP_ERROR(rclcpp::get_logger("PadControl"), "Service not available for releasing pad right");
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
    if (!m_acquire_client || !m_acquire_client->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_ERROR(rclcpp::get_logger("PadControl"), "Service not available for acquiring pad right");
        callback(false);
        return;
    }
    auto request = std::make_shared<pad_management_interfaces::srv::PadRightAcquire::Request>();
    request->name = m_prefix;
    request->timeout = timeout_seconds;

    auto result = m_acquire_client->async_send_request(request, 
        [this, callback](rclcpp::Client<pad_management_interfaces::srv::PadRightAcquire>::SharedFutureWithRequest response_future) {
            auto response = response_future.get().second;
            RCLCPP_INFO(rclcpp::get_logger("PadControl"), "Pad right acquired: %s", response->success ? "true" : "false");
            callback(response->success);
        }
    );
}

void PadControl::release_right_async(RightCallbackT && callback)
{
    if (!m_release_client || !m_release_client->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_ERROR(rclcpp::get_logger("PadControl"), "Service not available for releasing pad right");
        return callback(false);
    }
    auto request = std::make_shared<pad_management_interfaces::srv::PadRightRelease::Request>();
    request->name = m_prefix;

    auto result = m_release_client->async_send_request(request, 
        [this, callback](rclcpp::Client<pad_management_interfaces::srv::PadRightRelease>::SharedFutureWithRequest response_future) {
            auto response = response_future.get().second;
            RCLCPP_INFO(rclcpp::get_logger("PadControl"), "Pad right released: %s", response->success ? "true" : "false");
            callback(response->success);
        }
    );
}

bool PadControl::get_pad_circle_target(
    double timeout_seconds,
    Eigen::Vector3d & position, 
    Eigen::Vector3d & target_position)
{
    if (!m_circle_behaviour_client) return false;

    RCLCPP_INFO(rclcpp::get_logger("PadControl"), "Waiting for service for %s", m_prefix.c_str());
    if (!m_circle_behaviour_client->wait_for_service(std::chrono::milliseconds((long int)(timeout_seconds * 1000)))) {
        RCLCPP_ERROR(rclcpp::get_logger("PadControl"), "Service not available for getting pad circle target");
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger("PadControl"), "Requesting pad circle target for %s", m_prefix.c_str());

    auto request = std::make_shared<pad_management_interfaces::srv::PadCircleBehaviour::Request>();
    request->name = m_prefix;
    request->position = geometry_msgs::msg::Point();
    request->position.x = position.x();
    request->position.y = position.y();
    request->position.z = position.z();

    auto result = m_circle_behaviour_client->async_send_request(request);
    RCLCPP_INFO(rclcpp::get_logger("PadControl"), "Waiting for pad circle target response for %s", m_prefix.c_str());
    auto status = result.wait_for(std::chrono::milliseconds((long int)(timeout_seconds * 1000)));
    
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

std::string PadControl::get_frame_name() const
{
    return m_frame_name;
}

