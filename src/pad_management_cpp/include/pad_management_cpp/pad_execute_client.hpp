#pragma once

#include <chrono>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pad_management_interfaces/action/pad_execute.hpp"


class PadExecuteClient : public std::enable_shared_from_this<PadExecuteClient>
{
public:
    using PadExecuteActionT = pad_management_interfaces::action::PadExecute;
    using PadExecuteGoalHandleT = rclcpp_action::ClientGoalHandle<PadExecuteActionT>;
    using PadExecuteGoalHandlePtr = std::shared_ptr<PadExecuteGoalHandleT>;

    struct FeedbackUpdate
    {
        uint8_t status;
        geometry_msgs::msg::PoseStamped current_pose;
        double battery_percentage;
    };

    PadExecuteClient(
        const std::string & name,
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
        rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
        rclcpp::CallbackGroup::SharedPtr callback_group)
    : m_logger(node_logging_interface->get_logger().get_child("PadExecuteClient[" + name + "]"))
    {
        std::string action_name = name + "/pad_execute";
        m_action_client = rclcpp_action::create_client<PadExecuteActionT>(
            node_base_interface,
            node_graph_interface,
            node_logging_interface,
            node_waitables_interface,
            action_name,
            callback_group);

        RCLCPP_DEBUG(m_logger, "Created action client for %s", action_name.c_str());
    }

    bool is_finished() const { 
        return m_is_done; 
    }
    
    uint8_t result() const { 
        return m_result; 
    }

    std::deque<FeedbackUpdate> take_feedback()
    {
        std::deque<FeedbackUpdate> feedback;
        std::lock_guard<std::mutex> lock(m_feedback_mutex);
        feedback.swap(m_feedback_queue);
        return feedback;
    }

    bool wait_for_action_server_available()
    {
        const bool available = m_action_client->wait_for_action_server(std::chrono::milliseconds(100));
        if (!available) {
            RCLCPP_ERROR(m_logger, "Action server not available after waiting");
        }
        return available;
    }

    bool
    send_goal(
        const std::string & pad_name,
        uint8_t action)
    {
        RCLCPP_DEBUG(m_logger, "Sending goal to padflie %s with action %d", pad_name.c_str(), action);
        if (!m_action_client->action_server_is_ready()) {
            return false;
        }

        auto goal_msg = PadExecuteActionT::Goal();
        goal_msg.pad_name = pad_name;
        goal_msg.action = action;

        auto send_goal_options = rclcpp_action::Client<PadExecuteActionT>::SendGoalOptions();
        const auto self = shared_from_this();

        send_goal_options.goal_response_callback =
            [self](const PadExecuteGoalHandlePtr & goal_handle) {
                self->goal_response_callback(goal_handle);
            };
        send_goal_options.feedback_callback =
            [self](
                PadExecuteGoalHandlePtr goal_handle,
                const std::shared_ptr<const PadExecuteActionT::Feedback> feedback) {
                self->feedback_callback(goal_handle, feedback);
            };
        send_goal_options.result_callback =
            [self](const PadExecuteGoalHandleT::WrappedResult & result) {
                self->result_callback(result);
            };

        m_action_client->async_send_goal(goal_msg, send_goal_options);
        return true;
    }

    void goal_response_callback(const PadExecuteGoalHandlePtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(m_logger, "The padflie rejected the goal");
            m_is_done = true;
            m_result = PadExecuteGoalHandleT::Result::RESULT_UNKNOWN;
        } else {
            RCLCPP_DEBUG(m_logger, "The padflie accepted the goal");
        }
    }

    void feedback_callback(
        PadExecuteGoalHandlePtr goal_handle,
        const std::shared_ptr<const PadExecuteActionT::Feedback> feedback)
    {
        (void)goal_handle;

        FeedbackUpdate update;
        update.status = feedback->status;
        update.current_pose = feedback->current_pose;
        update.battery_percentage = feedback->battery_percentage;

        std::lock_guard<std::mutex> lock(m_feedback_mutex);
        m_feedback_queue.push_back(std::move(update));
    }
    
    void result_callback(const PadExecuteGoalHandleT::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_DEBUG(m_logger, "Padflie said: Goal succeeded: %s", result.result->reason.c_str());
        } else {
            RCLCPP_ERROR(m_logger, "Padflie said: Goal failed");
        }
        m_is_done = true;
        m_result = result.result->result;
    }
    
private:
    rclcpp::Logger m_logger; 

    bool m_is_done = false;
    uint8_t m_result = PadExecuteGoalHandleT::Result::RESULT_UNKNOWN;

    std::mutex m_feedback_mutex;
    std::deque<FeedbackUpdate> m_feedback_queue;

    std::shared_ptr<rclcpp_action::Client<PadExecuteActionT>> m_action_client;
};
