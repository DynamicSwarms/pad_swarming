#pragma once

#include "padflies_interfaces/action/pad_execute.hpp"

class PadExecuteServer
{
public:
    using PadExecuteActionT = padflies_interfaces::action::PadExecute;
    using PadExecuteGoalHandleT = rclcpp_action::ServerGoalHandle<PadExecuteActionT>;

    PadExecuteServer(
        const std::string & prefix,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
        std::shared_ptr<rclcpp::CallbackGroup> callback_group)
        : m_prefix(prefix)
        , m_logger(node_logging_interface->get_logger())
    {
        m_pad_execute_action_server = rclcpp_action::create_server<PadExecuteActionT>(
                node_base_interface,
                node_graph_interface,
                node_logging_interface,
                node_waitables_interface,
                prefix + "/pad_execute",
                std::bind(&PadExecuteServer::handle_pad_execute_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&PadExecuteServer::handle_pad_execute_cancel, this, std::placeholders::_1),
                std::bind(&PadExecuteServer::handle_pad_execute_accepted, this, std::placeholders::_1)
            );
    }

    void set_selected_pad_name(const std::string & pad_name) { m_selected_pad_name = pad_name; }

    void goal_received() { m_goal_received = true; }
    void goal_cancelled() { m_goal_cancelled = true; }

    void send_feedback(uint8_t status)
    {
        if (m_current_pad_execute_goal_handle) {
            auto feedback = std::make_shared<PadExecuteActionT::Feedback>();
            feedback->status = status;
            m_current_pad_execute_goal_handle->publish_feedback(feedback);
        }
    }

private: 
    std::string m_selected_pad_name = "";

    bool m_goal_cancelled = false;
    bool m_goal_received = false;


    rclcpp_action::GoalResponse handle_pad_execute_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PadExecuteActionT::Goal> goal)
    {
        // TODO: Check if the goal is of a selected pad, otherwise reject
        RCLCPP_INFO(m_logger, "Received goal request with pad name %s and action %d", goal->pad_name.c_str(), goal->action);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_pad_execute_cancel(
        const std::shared_ptr<PadExecuteGoalHandleT> goal_handle)
    {
        RCLCPP_INFO(m_logger, "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_pad_execute_accepted(const std::shared_ptr<PadExecuteGoalHandleT> goal_handle)
    {
        RCLCPP_INFO(m_logger, "Goal accepted, but no execution implemented.");
    }

};