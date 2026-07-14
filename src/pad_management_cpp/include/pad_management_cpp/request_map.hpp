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
        pad_management_cpp::IPadResourceManager & pad_resource_manager,
        int max_requests,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        rclcpp::Logger logger)
    : m_pad_resource_manager(pad_resource_manager)
    , m_max_requests(max_requests)
    , m_node_clock_interface(node_clock_interface)
    , m_logger(logger)
    {
    }

    virtual ~RequestMap() = default;

    bool fits_more_requests() const { return m_request_map.size() < static_cast<size_t>(m_max_requests); }

    bool has_name(const std::string & name)
    {
        std::lock_guard<std::mutex> lock(m_request_mutex);
        for (const auto & pair : m_request_map) {
            if (pair.second.name() == name) {
            return true;
            }
        }
        return false;
    }

    void add_request(const GoalHandlePtr goal_handle, std::shared_ptr<PadExecuteClient> pad_execute_client)
    {
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

        m_manage_requests_locked(true);
    }

    bool manage_requests()
    {
        std::lock_guard<std::mutex> lock(m_request_mutex);
        return m_manage_requests_locked();
    }

private: 
    bool m_manage_requests_locked(bool _publish_feedback = false)
    {
        bool publish_feedback = _publish_feedback;
        bool state_changed = false;
        for (auto it = m_request_map.begin(); it != m_request_map.end();) {
            auto & request = it->second;
            RequestUpdateResponse response = request.update();
            if (response.state_changed) publish_feedback = true;
            if (response.state_changed || response.is_finished || response.is_cancelled) state_changed = true;
            if (response.is_finished || response.is_cancelled) {
                RCLCPP_DEBUG(m_logger, "Request finished or cancelled with name %s, removing from map.", request.name().c_str());
                it = m_request_map.erase(it);
            } else {
                ++it;
            }
        }

        if (publish_feedback) {
            m_publish_feedback();
        }

        return state_changed;
    }


    void m_publish_feedback()
    { 
        for (const auto & pair : m_request_map)
        {
            pair.second.publish_feedback();
        }
        RCLCPP_DEBUG(m_logger, "Published feedback for %lu requests.", m_request_map.size());
    }

    pad_management_cpp::IPadResourceManager & m_pad_resource_manager;
    int m_max_requests;

    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_node_clock_interface;
    rclcpp::Logger m_logger;


    std::mutex m_request_mutex;
    RequestStore m_request_map;

};