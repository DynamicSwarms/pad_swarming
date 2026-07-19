#pragma once

#include <cstdint>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>

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

namespace pm = pad_management_cpp;

struct RequestUpdateResponse
{
    bool state_changed{false};
    bool is_finished{false};
    bool is_cancelled{false};
};

class Request
{
public:
    using PadRightControlActionT = pad_management_interfaces::action::PadRightControl;
    using PadRightControlGoalHandleT = rclcpp_action::ServerGoalHandle<PadRightControlActionT>;

    Request(
        rclcpp::Logger logger,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        const std::shared_ptr<PadRightControlGoalHandleT> & goal_handle,
        pad_management_cpp::IPadResourceManager & pad_resource_manager,
        std::shared_ptr<PadExecuteClient> pad_execute_client)
        :   m_logger(logger.get_child("[" + goal_handle->get_goal()->name + "]"))
        ,   m_clock(node_clock_interface->get_clock())
        ,   m_name(goal_handle->get_goal()->name)
        ,   m_request_time(m_clock->now())
        ,   m_max_wait_time(goal_handle->get_goal()->max_wait_time)
        ,   m_usage_time(goal_handle->get_goal()->usage_time)
        ,   m_goal_handle(goal_handle)
        ,   m_resource_manager(pad_resource_manager)
        ,   m_pad_execute_client(pad_execute_client)
    {
        try {
            std::string id_str = m_name.substr(7);
            m_id = std::stoi(id_str);
        } catch (const std::exception & e) {
            (void)e;
            RCLCPP_WARN(m_logger, "Failed to extract ID from node name: %s", m_name.c_str());
        }
        pm::AccessRequest req;
        req.id = m_id;
        req.action = m_goal_handle->get_goal()->action;
        req.max_wait_time = m_goal_handle->get_goal()->max_wait_time;
        req.usage_time = m_goal_handle->get_goal()->usage_time;
        req.battery_percentage = m_goal_handle->get_goal()->battery_percentage;
        req.current_pose = m_goal_handle->get_goal()->current_pose;

        req.request_time = m_request_time;
        m_access_handle = m_resource_manager.submit_access_request(req);
    }

    ~Request()
    {
        if (m_pad_execute_client->is_finished()) {
            auto result = std::make_shared<PadRightControlActionT::Result>();
            result->success = true;
            result->reason = "Completed successfully";
            m_goal_handle->succeed(result);

            pm::ExecuteResult exec_result;
            exec_result.result = m_pad_execute_client->result();
            m_resource_manager.notify_finished(m_access_handle, exec_result);
        } else if (m_goal_handle->is_canceling()) {
            const bool was_owner = m_state == RequestState::Acquired;

            auto result = std::make_shared<PadRightControlActionT::Result>();
            result->success = true;
            result->reason = "Canceled by client";
            m_goal_handle->canceled(result);

            RCLCPP_INFO(m_logger, "Canceling goal, was owner? %s", was_owner ? "Yes" : "No");

            m_resource_manager.cancel(m_access_handle);
        } else if (m_state != RequestState::FinishedAndResponded) {
            auto result = std::make_shared<PadRightControlActionT::Result>();
            result->success = false;
            result->reason = "Request destroyed before completion";
            m_goal_handle->abort(result);
            m_resource_manager.cancel(m_access_handle);
        }
    }

    
    

    RequestUpdateResponse update()
    {
        RequestState previous_state = m_state;

        for (auto & feedback : m_pad_execute_client->take_feedback()) {
            pm::ExecuteUpdate update;
            update.status = feedback.status;
            update.current_pose = std::move(feedback.current_pose);
            update.battery_percentage = feedback.battery_percentage;
            m_resource_manager.notify_update(m_access_handle, update);
        }

        if (m_state == RequestState::Waiting) {
            pm::AccessResponse resp = m_resource_manager.query_request_status(m_access_handle);
            m_expected_wait_time = resp.wait_time;
            if (resp.result == pm::AccessResponse::Result::ACCEPTED) {
                m_acquire_time = m_clock->now();
                m_state = RequestState::Acquired;
                RCLCPP_DEBUG(m_logger, "Acquired rights.");
            } else if (resp.result == pm::AccessResponse::Result::REJECTED) {
                auto result = std::make_shared<PadRightControlActionT::Result>();
                result->success = false;
                result->reason = resp.message.empty() ? "Request rejected" : resp.message;
                m_goal_handle->abort(result);
                m_state = RequestState::FinishedAndResponded;
            } else if (resp.result == pm::AccessResponse::Result::PENDING) {
                auto now = m_clock->now();
                if (m_request_time + m_max_wait_time < now) {
                    auto result = std::make_shared<PadRightControlActionT::Result>();
                    result->success = false;
                    result->reason = "Request timed out";
                    m_goal_handle->abort(result);
                    m_state = RequestState::FinishedAndResponded;
                    RCLCPP_INFO(m_logger, "Request timed out.");
                }
            } 
        } else if (m_state == RequestState::Acquired) {
            if (m_pad_execute_client->is_finished()) {
                m_state = RequestState::Finished;
                RCLCPP_DEBUG(m_logger, "Execution finished.");
            }
        }

        RequestUpdateResponse response;
        response.state_changed = previous_state != m_state;
        response.is_finished = (m_state == RequestState::Finished || 
                                m_state == RequestState::FinishedAndResponded || 
                                m_pad_execute_client->is_finished());
        response.is_cancelled = m_goal_handle->is_canceling();
        return response;
    }

    void publish_feedback() const
    {
        rclcpp::Duration expected_wait_time = m_expected_wait_time;
        auto feedback = std::make_shared<PadRightControlActionT::Feedback>();
        if (m_state == RequestState::Acquired) {
            feedback->status = PadRightControlActionT::Feedback::STATUS_ACQUIRED_RIGHT;
            auto time_held = m_clock->now() - m_acquire_time;
            feedback->time_remaining = expected_wait_time;

            feedback->target_pose = geometry_msgs::msg::PoseStamped();
            m_resource_manager.get_associated_position(m_id, feedback->target_pose);
        } else {
            feedback->status = PadRightControlActionT::Feedback::STATUS_WAITING_FOR_RIGHT;
            feedback->time_remaining = expected_wait_time;
        }

        m_goal_handle->publish_feedback(feedback);
        RCLCPP_DEBUG(
            m_logger,
            "Publishing feedback: status %i, time_remaining: %f",
            feedback->status,
            duration_to_seconds(feedback->time_remaining));
    }

    const std::string & name() const { 
        return m_name; 
    }
    
    
private:
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::Clock> m_clock;

    std::string m_name;
    rclcpp::Time m_request_time;
    rclcpp::Duration m_max_wait_time;
    rclcpp::Duration m_usage_time;
    std::shared_ptr<PadRightControlGoalHandleT> m_goal_handle;
    rclcpp::Time m_acquire_time;
    
    pm::IPadResourceManager & m_resource_manager;
    pm::AccessHandle m_access_handle;
    std::shared_ptr<PadExecuteClient> m_pad_execute_client;

    uint8_t m_id = 0;


    enum RequestState
    {
        Waiting,
        Acquired,
        Finished,
        FinishedAndResponded
    };
    RequestState m_state = Waiting;
    rclcpp::Duration m_expected_wait_time{rclcpp::Duration::from_seconds(0.0)};

};
