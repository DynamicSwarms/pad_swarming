#pragma once

#include "rclcpp_action/rclcpp_action.hpp"
#include "pad_management_interfaces/action/pad_right_control.hpp"

class PadRightClient
{
public:
    using PadRightControlActionT = pad_management_interfaces::action::PadRightControl;
    using PadRightControlGoalHandleT = rclcpp_action::ClientGoalHandle<PadRightControlActionT>;

    PadRightClient(
        const std::string & prefix,
        const std::string & pad_name,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
        std::shared_ptr<rclcpp::CallbackGroup> callback_group,
        rclcpp::Logger parent_logger)
        : m_prefix(prefix)
        , m_logger(parent_logger.get_child("PadRightClient[" + pad_name + "]"))
    {
        m_pad_right_control_action_client = rclcpp_action::create_client<PadRightControlActionT>(
                node_base_interface,
                node_graph_interface,
                node_logging_interface,
                node_waitables_interface,
                pad_name + "/pad_right_control",
                callback_group);
    }

    bool is_action_server_available(std::chrono::milliseconds timeout = std::chrono::seconds(5)) {
        return m_pad_right_control_action_client->wait_for_action_server(timeout);
    }

    void send_request(uint8_t action) 
    {
        auto goal_msg = PadRightControlActionT::Goal();
        goal_msg.action = action;
        goal_msg.name = m_prefix;
        auto send_goal_options = rclcpp_action::Client<PadRightControlActionT>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&PadRightClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(&PadRightClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&PadRightClient::result_callback, this, std::placeholders::_1);
        m_pad_right_control_action_client->async_send_goal(goal_msg, send_goal_options);
    }

    bool goal_responded() { return m_goal_responded; }
    bool goal_accepted() { return m_goal_accepted; }

    bool has_right(){return m_right_acquired;}

    bool received_result() { return m_received_result; }
    bool result_success() { return m_result_success; }

private:
    void goal_response_callback(const typename PadRightControlGoalHandleT::SharedPtr & goal_handle) 
    {
        m_current_goal_handle = goal_handle;
        m_goal_accepted = !!goal_handle;
        m_goal_responded = true;
        
        if (!goal_handle) {
            RCLCPP_ERROR(m_logger, "Goal rejected");
        } else {
            RCLCPP_INFO(m_logger, "Goal accepted");
        }
    }

    void feedback_callback(
        typename PadRightControlGoalHandleT::SharedPtr goal_handle,
        const std::shared_ptr<const PadRightControlActionT::Feedback> feedback)
    {
        (void)goal_handle;
        m_right_acquired = feedback->status == PadRightControlActionT::Feedback::STATUS_ACQUIRED_RIGHT;

        if (feedback->status == PadRightControlActionT::Feedback::STATUS_WAITING_FOR_RIGHT)
            RCLCPP_INFO(m_logger, "FBD: Pad is waiting for right to be acquired...");
        else if (feedback->status == PadRightControlActionT::Feedback::STATUS_ACQUIRED_RIGHT)
            RCLCPP_INFO(m_logger, "FBD: Acquired right!");
    }

    void result_callback(const typename PadRightControlGoalHandleT::WrappedResult & result) 
    {
        m_result_success = result.code == rclcpp_action::ResultCode::SUCCEEDED;
        m_received_result = true;

        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(m_logger, "Pad right control action succeeded");
        } else {
            RCLCPP_ERROR(m_logger, "Pad right control action failed with code %d", result.code);
        }
    }



private: 
    std::string m_prefix;
    rclcpp::Logger m_logger;

    std::shared_ptr<rclcpp_action::Client<PadRightControlActionT>> m_pad_right_control_action_client;

private: 
    std::shared_ptr<PadRightControlGoalHandleT> m_current_goal_handle;
    bool m_goal_responded = false;
    bool m_goal_accepted = false;
    bool m_right_acquired = false;
    
    bool m_received_result = false;
    bool m_result_success = false;
};
    