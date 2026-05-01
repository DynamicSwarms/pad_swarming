#include "pad_management_cpp/pad_execute_client.hpp"

PadExecuteClient::PadExecuteClient(
    std::string & name,
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
    rclcpp::node_interfaces::NodeGraphInterface::SharedPtr node_graph_interface,
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
    rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
    rclcpp::CallbackGroup::SharedPtr callback_group)
: m_logger(node_logging_interface->get_logger().get_child("PadExecuteClient"))
{
    m_action_client = rclcpp_action::create_client<ActionT>(
        node_base_interface,
        node_graph_interface,
        node_logging_interface,
        node_waitables_interface,
        "/" + name + "/pad_execute",
        callback_group);
}


void 
PadExecuteClient::send_goal(
    const std::string & pad_name,
    uint8_t action)
{
    if (!this->m_action_client->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(rclcpp::get_logger("PadExecuteClient"), "Action server not available after waiting");
        return;
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
}

void  
PadExecuteClient::goal_response_callback(const GoalHandlePtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(rclcpp::get_logger("PadExecuteClient"), "Goal rejected");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("PadExecuteClient"), "Goal accepted");
    }
}

void 
PadExecuteClient::feedback_callback(
    GoalHandlePtr goal_handle,
    const std::shared_ptr<const ActionT::Feedback> feedback)
{
    RCLCPP_INFO(rclcpp::get_logger("PadExecuteClient"), "Received feedback: %d", feedback->status);
}

void 
PadExecuteClient::result_callback(const GoalHandleT::WrappedResult & result)
{
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(rclcpp::get_logger("PadExecuteClient"), "Goal succeeded: %s", result.result->reason.c_str());
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("PadExecuteClient"), "Goal failed with code");// %d", result.status);
    }
}