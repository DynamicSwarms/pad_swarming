#include "pad_management_cpp/pad_execute_client.hpp"

PadExecuteClient::PadExecuteClient(
    std::string & name,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    rclcpp::CallbackGroup::SharedPtr callback_group)
: m_logger(node_logging_interface->get_logger().get_child("PadExecuteClient[" + name + "]"))
{
    std::string action_name = name + "/pad_execute";
    m_action_client = rclcpp_action::create_client<ActionT>(
        node_base_interface,
        node_graph_interface,
        node_logging_interface,
        node_waitables_interface,
        action_name,
        callback_group);

    RCLCPP_DEBUG(m_logger, "Created action client for %s", action_name.c_str());
}

bool 
PadExecuteClient::wait_for_action_server_available()
{
    return m_action_client->wait_for_action_server(std::chrono::milliseconds(100));
    RCLCPP_ERROR(m_logger, "Action server not available after waiting");
}


bool 
PadExecuteClient::send_goal(
    const std::string & pad_name,
    uint8_t action)
{
    RCLCPP_INFO(m_logger, "Sending goal to padflie %s with action %d", pad_name.c_str(), action);
    if (!this->m_action_client->action_server_is_ready()) {
        return false;
    }

    auto goal_msg = ActionT::Goal();
    goal_msg.pad_name = pad_name;
    goal_msg.action = action;

    auto send_goal_options = rclcpp_action::Client<ActionT>::SendGoalOptions();

    send_goal_options.goal_response_callback = std::bind(
        &PadExecuteClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(
        &PadExecuteClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &PadExecuteClient::result_callback, this, std::placeholders::_1);

    m_action_client->async_send_goal(goal_msg, send_goal_options);
    return true;
}

void  
PadExecuteClient::goal_response_callback(const GoalHandlePtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(m_logger, "The padflie rejected the goal");
    } else {
        RCLCPP_INFO(m_logger, "The padflie accepted the goal");
    }
}

void 
PadExecuteClient::feedback_callback(
    GoalHandlePtr goal_handle,
    const std::shared_ptr<const ActionT::Feedback> feedback)
{
    RCLCPP_INFO(m_logger, "Received feedback from padflie: %d", feedback->status);
}

void 
PadExecuteClient::result_callback(const GoalHandleT::WrappedResult & result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(m_logger, "Padflie said: Goal succeeded: %s", result.result->reason.c_str());
    } else {
        RCLCPP_ERROR(m_logger, "Padflie said: Goal failed with code");// %d", result.status);
    }
    m_is_done = true;
    m_result = result.result->result;
}