#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pad_management_interfaces/action/pad_right_control.hpp"

rclcpp::Duration duration_from_seconds(float seconds);

class Request
{
public:
    using ActionT = pad_management_interfaces::action::PadRightControl;
    using GoalHandleT = rclcpp_action::ServerGoalHandle<ActionT>;

    Request(
        rclcpp::Logger logger,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        const std::shared_ptr<GoalHandleT> & goal_handle,
        std::mutex & pad_mutex);

    const std::string & name() const;
    bool owns_lock() const;

    bool try_acquire();
    bool hold_time_exceeded(const rclcpp::Duration & max_hold_time) const;

    void publish_feedback(
        const rclcpp::Duration & expected_wait_time, 
        const rclcpp::Duration & max_hold_time) const;

    rclcpp::Time request_time() const { return m_request_time; };
    rclcpp::Time acquire_time() const {return m_acquire_time;};
    rclcpp::Duration usage_time() const {return m_usage_time;};
    

    bool check_cancel();

private:
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::Clock> m_clock;

    std::string m_name;
    rclcpp::Time m_request_time;
    rclcpp::Duration m_max_wait_time;
    rclcpp::Duration m_usage_time;
    std::shared_ptr<GoalHandleT> m_goal_handle;
    std::unique_lock<std::mutex> m_pad_lock;
    rclcpp::Time m_acquire_time;
};