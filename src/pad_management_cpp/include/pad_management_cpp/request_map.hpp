#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <type_traits>

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
    void add_request(const GoalHandlePtr goal_handle);
    bool manage_requests();

protected:
    virtual std::vector<rclcpp_action::GoalUUID> m_order_request_map(const RequestStore & request_map);
    
private: 

    std::vector<rclcpp::Duration> m_get_expected_wait_times(
        const std::vector<rclcpp_action::GoalUUID> & ordered_uuids);

    bool m_select_new_owner();
    void m_publish_feedback();
    void m_check_cancelations();
    void m_check_timeouts();

    IPadResourceManager & m_pad_resource_manager;
    int m_max_requests;
    rclcpp::Duration m_max_hold_time; 

    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_node_clock_interface;
    rclcpp::Logger m_logger;


    std::mutex m_request_mutex;
    RequestStore m_request_map;

};