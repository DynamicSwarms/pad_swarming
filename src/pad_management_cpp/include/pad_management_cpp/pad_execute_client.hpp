#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pad_management_interfaces/action/pad_execute.hpp"


class PadExecuteClient
{
public:
    using ActionT = pad_management_interfaces::action::PadExecute;
    using GoalHandleT = rclcpp_action::ClientGoalHandle<ActionT>;
    using GoalHandlePtr = std::shared_ptr<GoalHandleT>;

    PadExecuteClient(
        std::string & name,
        rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
        rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
        rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
        rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
        rclcpp::CallbackGroup::SharedPtr callback_group
    );

    bool is_finished() const { return m_is_done; }
    uint8_t result() const { return m_result; }

    void send_goal(
        const std::string & pad_name,
        uint8_t action);

    void goal_response_callback(const GoalHandlePtr & goal_handle);

    void feedback_callback(
        GoalHandlePtr goal_handle,
        const std::shared_ptr<const ActionT::Feedback> feedback);
    
    void result_callback(const GoalHandleT::WrappedResult & result);
private:
    rclcpp::Logger m_logger; 

    bool m_is_done = false;
    uint8_t m_result = GoalHandleT::Result::RESULT_UNKNOWN;

    std::shared_ptr<rclcpp_action::Client<ActionT>> m_action_client;
};