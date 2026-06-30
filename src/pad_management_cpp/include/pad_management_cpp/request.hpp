#pragma once

#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pad_management_interfaces/action/pad_right_control.hpp"
#include "pad_management_cpp/I_pad_resource_manager.hpp"
#include "pad_management_cpp/pad_execute_client.hpp"

inline rclcpp::Duration duration_from_seconds(double seconds)
{
    int32_t sec = static_cast<int32_t>(seconds);
    uint32_t nanosec = static_cast<uint32_t>((seconds - sec) * 1e9);
    return rclcpp::Duration(sec, nanosec);
}

inline float duration_to_seconds(const rclcpp::Duration & duration)
{
    return duration.seconds();
}

class Request
{
public:
    using PadRightControlActionT = pad_management_interfaces::action::PadRightControl;
    using PadRightControlGoalHandleT = rclcpp_action::ServerGoalHandle<PadRightControlActionT>;

    Request(
        rclcpp::Logger logger,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        const std::shared_ptr<PadRightControlGoalHandleT> & goal_handle,
        IPadResourceManager & pad_resource_manager,
        std::shared_ptr<PadExecuteClient> pad_execute_client);

    ~Request();

    const std::string & name() const { return m_name; }
    bool is_finished() const {return m_pad_execute_client->is_finished(); }
    uint8_t result() const {return m_pad_execute_client->result(); }
    bool owns_lock() const;

    bool update();

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
    std::shared_ptr<PadRightControlGoalHandleT> m_goal_handle;
    rclcpp::Time m_acquire_time;
    
    IPadResourceManager & m_resource_manager;
    std::shared_ptr<PadExecuteClient> m_pad_execute_client;

    uint8_t m_id = 0;


    enum RequestState
    {
        Waiting,
        Acquired,
        Finished
    };
    RequestState m_state = Waiting;

    bool m_executing = false;
};

inline Request::Request(
    rclcpp::Logger logger,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    const std::shared_ptr<PadRightControlGoalHandleT> & goal_handle,
    IPadResourceManager & pad_resource_manager,
    std::shared_ptr<PadExecuteClient> pad_execute_client)
:   m_logger(logger.get_child("[" + goal_handle->get_goal()->name + "]")),
    m_clock(node_clock_interface->get_clock()),
    m_name(goal_handle->get_goal()->name),
    m_request_time(m_clock->now()),
    m_max_wait_time(duration_from_seconds(goal_handle->get_goal()->max_wait_time)),
    m_usage_time(duration_from_seconds(goal_handle->get_goal()->usage_time)),
    m_goal_handle(goal_handle),
    m_resource_manager(pad_resource_manager),
    m_pad_execute_client(pad_execute_client)
{
    try {
        std::string id_str = m_name.substr(7);
        m_id = std::stoi(id_str);
    } catch (const std::exception & e) {
        (void)e;
        RCLCPP_WARN(m_logger, "Failed to extract ID from node name: %s", m_name.c_str());
    }
}

inline Request::~Request()
{
    RCLCPP_INFO(m_logger, "Destroying request object.");

    if (is_finished()) {
        auto result = std::make_shared<PadRightControlActionT::Result>();
        result->success = true;
        result->reason = "Completed successfully";
        m_goal_handle->succeed(result);
    }

    if (m_executing) {
        m_resource_manager.release(m_id, m_pad_execute_client->result());
    }
}

inline bool Request::update()
{
    RequestState previous_state = m_state;

    if (m_state == RequestState::Waiting) {
        RequestData data;
        data.id = m_id;
        data.action = m_goal_handle->get_goal()->action;

        AdmissionResponse resp = m_resource_manager.admit_request(data);
        if (resp.result == AdmissionResponse::Result::ACCEPTED) {
            m_executing = true;
            m_acquire_time = m_clock->now();
            m_resource_manager.start_execution(data);
            m_state = RequestState::Acquired;
            RCLCPP_INFO(m_logger, "Acquired rights.");
        } else if (resp.result == AdmissionResponse::Result::REJECTED) {
            auto result = std::make_shared<PadRightControlActionT::Result>();
            result->success = false;
            result->reason = resp.message.empty() ? "Request rejected" : resp.message;
            m_goal_handle->abort(result);
            m_state = RequestState::Finished;
            RCLCPP_INFO(m_logger, "Request rejected.");
        }
    } else if (m_state == RequestState::Acquired) {
        if (m_pad_execute_client->is_finished()) {
            m_state = RequestState::Finished;
            RCLCPP_INFO(m_logger, "Execution finished.");
        }
    }

    return previous_state != m_state;
}

inline bool Request::check_cancel()
{
    if (m_goal_handle->is_canceling()) {
        const bool was_owner = owns_lock();

        auto result = std::make_shared<PadRightControlActionT::Result>();
        result->success = true;
        result->reason = "Canceled by client";
        m_goal_handle->canceled(result);

        RCLCPP_INFO(m_logger, "Canceling goal, was owner? %s", was_owner ? "Yes" : "No");
        return true;
    }
    return false;
}

inline bool Request::owns_lock() const
{
    return m_executing;
}

inline bool Request::try_acquire()
{
    if (!m_executing && m_resource_manager.try_lock(m_id, m_goal_handle->get_goal()->action)) {
        m_executing = true;
        m_acquire_time = m_clock->now();
        RCLCPP_INFO(m_logger, "Acquired lock.");
        return true;
    }
    return false;
}

inline bool Request::hold_time_exceeded(const rclcpp::Duration & max_hold_time) const
{
    if (m_executing && ((m_clock->now() - m_acquire_time) >= max_hold_time)) {
        auto result = std::make_shared<PadRightControlActionT::Result>();
        result->success = false;
        result->reason = "Hold time exceeded";
        m_goal_handle->abort(result);

        RCLCPP_INFO(m_logger, "Hold time exceeded, aborting goal.");
        return true;
    }
    return false;
}

inline void Request::publish_feedback(
    const rclcpp::Duration & expected_wait_time,
    const rclcpp::Duration & max_hold_time) const
{
    auto feedback = std::make_shared<PadRightControlActionT::Feedback>();
    if (owns_lock()) {
        feedback->status = PadRightControlActionT::Feedback::STATUS_ACQUIRED_RIGHT;
        auto time_held = m_clock->now() - m_acquire_time;
        feedback->time_remaining = duration_to_seconds(max_hold_time - time_held);

        feedback->target_pose = geometry_msgs::msg::PoseStamped();
        m_resource_manager.get_associated_position(m_id, feedback->target_pose);
    } else {
        feedback->status = PadRightControlActionT::Feedback::STATUS_WAITING_FOR_RIGHT;
        feedback->time_remaining = duration_to_seconds(expected_wait_time);
    }

    m_goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(
        m_logger,
        "Publishing feedback: status %i, time_remaining: %f",
        feedback->status,
        feedback->time_remaining);
}
