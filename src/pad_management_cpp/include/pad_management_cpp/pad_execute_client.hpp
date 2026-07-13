#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pad_management_interfaces/action/pad_execute.hpp"


class PadExecuteClient : public std::enable_shared_from_this<PadExecuteClient>
{
public:
    using PadExecuteActionT = pad_management_interfaces::action::PadExecute;
    using PadExecuteGoalHandleT = rclcpp_action::ClientGoalHandle<PadExecuteActionT>;
    using PadExecuteGoalHandlePtr = std::shared_ptr<PadExecuteGoalHandleT>;

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
        RCLCPP_INFO(m_logger, "Sending goal to padflie %s with action %d", pad_name.c_str(), action);
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
            RCLCPP_INFO(m_logger, "The padflie accepted the goal");
        }
    }

    void feedback_callback(
        PadExecuteGoalHandlePtr goal_handle,
        const std::shared_ptr<const PadExecuteActionT::Feedback> feedback)
    {
            (void)goal_handle;
            RCLCPP_DEBUG(m_logger, "Received feedback from padflie: %d", feedback->status);

            if (m_feedback_callback) {
                m_feedback_callback(feedback->status, feedback->current_pose);
            }
    }
    void add_feedback_callback(std::function<void(uint8_t, geometry_msgs::msg::PoseStamped)> callback)
    {
        m_feedback_callback = callback;
    }
    
    void result_callback(const PadExecuteGoalHandleT::WrappedResult & result)
    {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(m_logger, "Padflie said: Goal succeeded: %s", result.result->reason.c_str());
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

    std::function<void(uint8_t, geometry_msgs::msg::PoseStamped)> m_feedback_callback;

    std::shared_ptr<rclcpp_action::Client<PadExecuteActionT>> m_action_client;
};
