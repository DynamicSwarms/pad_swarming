#include "pad_management_cpp/request_map.hpp"
#include "pad_management_cpp/request.hpp"
#include "pad_management_cpp/pad_resource_manager.hpp"

RequestMap::RequestMap(
    IPadResourceManager & pad_resource_manager,
    int max_requests,
    rclcpp::Duration max_hold_time,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface,
    rclcpp::Logger logger)
    : m_pad_resource_manager(pad_resource_manager)
    , m_max_requests(max_requests)
    , m_max_hold_time(max_hold_time)
    , m_node_clock_interface(clock_interface)
    , m_logger(logger)
{
}

bool RequestMap::has_name(const std::string & name)
{
  std::lock_guard<std::mutex> lock(m_request_mutex);
  for (const auto & pair : m_request_map) {
    if (pair.second.name() == name) {
      return true;
    }
  }
  return false;
}

void RequestMap::add_request(
    const GoalHandlePtr goal_handle, 
    std::shared_ptr<PadExecuteClient> pad_execute_client)
{
    RCLCPP_INFO(m_logger, "Adding request with name %s", goal_handle->get_goal()->name.c_str());
    const auto uuid = goal_handle->get_goal_id();


    std::lock_guard<std::mutex> lock(m_request_mutex);

    m_request_map.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(uuid),
        std::forward_as_tuple(
            m_logger,
            m_node_clock_interface,
            goal_handle,
            m_pad_resource_manager, 
            pad_execute_client
        )
    );
    m_publish_feedback();
}

bool RequestMap::manage_requests()
{
    std::lock_guard<std::mutex> lock(m_request_mutex);
    m_check_cancelations();
    m_check_timeouts();
    m_check_done_status();

    bool new_owner_selected = false;
    if ((new_owner_selected = m_select_new_owner())) {
        m_publish_feedback();
    }
    return new_owner_selected;
}

bool RequestMap::m_select_new_owner()
{
    std::vector<GoalUUID> ordered_uuids = m_order_request_map(m_request_map);
    for (const auto & uuid : ordered_uuids) {
        auto & request = m_request_map.at(uuid);
        if (request.try_acquire()) {
            return true;
        }
    }
    return false;
}

void RequestMap::m_publish_feedback()
{
    auto feedback = std::make_shared<ActionT::Feedback>();

    std::vector<GoalUUID> ordered_uuids = m_order_request_map(m_request_map);
    std::vector<rclcpp::Duration> wait_times = m_get_expected_wait_times(ordered_uuids);
    for (size_t i = 0; i < ordered_uuids.size(); ++i) {
        const auto & uuid = ordered_uuids[i];
        auto & request = m_request_map.at(uuid);
        request.publish_feedback(wait_times[i], m_max_hold_time);
    }

    RCLCPP_INFO(m_logger, "Published feedback for %lu requests.", ordered_uuids.size());

}

void RequestMap::m_check_cancelations()
{
  for (auto it = m_request_map.begin(); it != m_request_map.end();) {
    auto & request = it->second;
    if (request.check_cancel()) {
        RCLCPP_INFO(m_logger, "Request canceled with name %s, removing from map.", request.name().c_str());
      it = m_request_map.erase(it);
    } else {
      ++it;
    }
  }
}

void RequestMap::m_check_done_status()
{
    for (auto it = m_request_map.begin(); it != m_request_map.end();) {
        auto & request = it->second;
        if (request.is_finished()) {
            RCLCPP_INFO(m_logger, "Request finished with name %s, removing from map.", request.name().c_str());
            it = m_request_map.erase(it);
        } else {
            ++it;
        }
    }
}

void RequestMap::m_check_timeouts()
{
    const auto now = m_node_clock_interface->get_clock()->now();

    for (auto it = m_request_map.begin(); it != m_request_map.end();) {
        auto & request = it->second;
        if (request.hold_time_exceeded(m_max_hold_time)) {
                    RCLCPP_INFO(m_logger, "Request timeout with name %s, removing from map.", request.name().c_str());

            it = m_request_map.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<rclcpp::Duration>
RequestMap::m_get_expected_wait_times(
    const std::vector<rclcpp_action::GoalUUID> & ordered_uuids)
{
    auto now = m_node_clock_interface->get_clock()->now();
    rclcpp::Duration expected_wait_time(0, 0);

    std::vector<rclcpp::Duration> wait_times;
    for (const auto & uuid : ordered_uuids) {
        const auto & r = m_request_map.at(uuid);
        wait_times.push_back(expected_wait_time);
        if (r.owns_lock()) {
            rclcpp::Duration time_held = now - r.acquire_time();
            rclcpp::Duration remaining_hold_time = r.usage_time() - time_held;
            rclcpp::Duration time_left = std::min(m_max_hold_time, remaining_hold_time);
            expected_wait_time = expected_wait_time + time_left;
        } else {
            expected_wait_time = expected_wait_time + r.usage_time();
        }
    }
    return wait_times;
}

std::vector<rclcpp_action::GoalUUID>
RequestMap::m_order_request_map(const RequestStore & request_map)
{
    std::vector<rclcpp_action::GoalUUID> uuids;
    uuids.reserve(request_map.size());

    // Collect all UUIDs
    for (const auto & kv : request_map) {
        uuids.push_back(kv.first);
    }

    // Sort by acquire_time (earliest first)
    std::sort(uuids.begin(), uuids.end(),
        [&request_map](const GoalUUID & a, const GoalUUID & b)
    {
        const auto & ra = request_map.at(a);
        const auto & rb = request_map.at(b);

        return ra.request_time() < rb.request_time();
    });

    return uuids;
}