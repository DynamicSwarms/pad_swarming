#pragma once

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pad_management_interfaces/action/pad_right_control.hpp"

#include "pad_management_cpp/request.hpp"

class RequestMap
{
public:
    using ActionT = pad_management_interfaces::action::PadRightControl;
    using GoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>;
    using GoalHandlePtr = std::shared_ptr<GoalHandleT>;
    using GoalUUID = rclcpp_action::GoalUUID;

    using RequestStore = std::unordered_map<GoalUUID, Request>;

    explicit RequestMap(
        IPadResourceManager & pad_resource_manager,
        int max_requests,
        rclcpp::Duration max_hold_time,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        rclcpp::Logger logger
    );
    virtual ~RequestMap() = default;

    std::string get_current_holder()
    {
        std::lock_guard<std::mutex> lock(m_request_mutex);
        for (const auto & pair : m_request_map) {
            if (pair.second.owns_lock()) {
                return pair.second.name();
            }
        }
        return "";
    }

    bool fits_more_requests() const { return m_request_map.size() < static_cast<size_t>(m_max_requests); }

    bool has_name(const std::string & name);
    void add_request(const GoalHandlePtr goal_handle, std::shared_ptr<PadExecuteClient> pad_execute_client);
    bool manage_requests();

    
protected:
    virtual std::vector<rclcpp_action::GoalUUID> m_order_request_map(const RequestStore & request_map);
    
private: 

    std::vector<rclcpp::Duration> m_get_expected_wait_times(
        const std::vector<rclcpp_action::GoalUUID> & ordered_uuids);

    /**
     * Update requests. Returns true if any request state has changed.
     */
    bool update_requests();


    bool m_select_new_owner();
    void m_publish_feedback();
    void m_check_cancelations();
    void m_check_timeouts();
    void m_check_done_status();


    IPadResourceManager & m_pad_resource_manager;
    int m_max_requests;
    rclcpp::Duration m_max_hold_time; 

    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_node_clock_interface;
    rclcpp::Logger m_logger;


    std::mutex m_request_mutex;
    RequestStore m_request_map;

};

inline RequestMap::RequestMap(
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

inline bool RequestMap::has_name(const std::string & name)
{
  std::lock_guard<std::mutex> lock(m_request_mutex);
  for (const auto & pair : m_request_map) {
    if (pair.second.name() == name) {
      return true;
    }
  }
  return false;
}

inline void RequestMap::add_request(
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

inline bool RequestMap::manage_requests()
{
    std::lock_guard<std::mutex> lock(m_request_mutex);

    m_check_cancelations();
    m_check_timeouts();
    m_check_done_status();

    bool state_changed = update_requests();
    if (state_changed) {
        m_publish_feedback();
    }

    return state_changed;
}

inline bool RequestMap::update_requests()
{
    bool state_changed = false;
    for (auto & pair : m_request_map) {
        auto & request = pair.second;
        if (request.update()) {
            state_changed = true;
        }
    }
    return state_changed;
}

inline bool RequestMap::m_select_new_owner()
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

inline void RequestMap::m_publish_feedback()
{
    std::vector<GoalUUID> ordered_uuids = m_order_request_map(m_request_map);
    std::vector<rclcpp::Duration> wait_times = m_get_expected_wait_times(ordered_uuids);
    for (size_t i = 0; i < ordered_uuids.size(); ++i) {
        const auto & uuid = ordered_uuids[i];
        auto & request = m_request_map.at(uuid);
        request.publish_feedback(wait_times[i], m_max_hold_time);
    }

    RCLCPP_INFO(m_logger, "Published feedback for %lu requests.", ordered_uuids.size());
}

inline void RequestMap::m_check_cancelations()
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

inline void RequestMap::m_check_done_status()
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

inline void RequestMap::m_check_timeouts()
{
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

inline std::vector<rclcpp::Duration>
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

inline std::vector<rclcpp_action::GoalUUID>
RequestMap::m_order_request_map(const RequestStore & request_map)
{
    std::vector<rclcpp_action::GoalUUID> uuids;

    uuids.reserve(request_map.size());
    for (const auto & kv : request_map) {
        uuids.push_back(kv.first);
    }

    std::sort(uuids.begin(), uuids.end(),
        [&request_map](const GoalUUID & a, const GoalUUID & b)
    {
        const auto & ra = request_map.at(a);
        const auto & rb = request_map.at(b);

        return ra.request_time() < rb.request_time();
    });

    return uuids;
}
