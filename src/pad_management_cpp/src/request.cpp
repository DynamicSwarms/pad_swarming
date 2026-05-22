#include "pad_management_cpp/request.hpp"

rclcpp::Duration duration_from_seconds(float seconds)
{
    int32_t sec = static_cast<int32_t>(seconds);
    uint32_t nanosec = static_cast<uint32_t>((seconds - sec) * 1e9);
    return rclcpp::Duration(sec, nanosec);
}

float duration_to_seconds(const rclcpp::Duration & duration)
{
    return duration.seconds();
}

Request::Request(
    rclcpp::Logger logger,
    rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
    const std::shared_ptr<PadRightControlGoalHandleT> & goal_handle,
    IPadResourceManager & pad_resource_manager, 
    std::shared_ptr<PadExecuteClient> pad_execute_client)
:   m_logger(logger.get_child("[" + goal_handle->get_goal()->name + "]")),
    m_clock(node_clock_interface->get_clock()),
    m_name( goal_handle->get_goal()->name),
    m_request_time(m_clock->now()),
    m_max_wait_time(duration_from_seconds(goal_handle->get_goal()->max_wait_time)),
    m_usage_time(duration_from_seconds(goal_handle->get_goal()->usage_time)),
    m_goal_handle(goal_handle),
    m_resource_manager(pad_resource_manager),
    m_pad_execute_client(pad_execute_client)
{
    try {
        std::string id_str = m_name.substr(8); // Assuming name is like "/padflieID"
        m_id = std::stoi(id_str);
    } catch (const std::exception& e) {
        RCLCPP_WARN(m_logger, "Failed to extract ID from node name: %s", m_name.c_str());
    }

    pad_execute_client->send_goal(m_name, pad_management_interfaces::action::PadExecute::Goal::ACTION_TAKEOFF);
}

Request::~Request()
{
    RCLCPP_INFO(m_logger, "Destroying request object.");
    
    if (is_finished()) 
    {
        auto result = std::make_shared<PadRightControlActionT::Result>();
        result->success = true;
        result->reason = "Completed successfully";
        m_goal_handle->succeed(result);
    }


    if (m_executing) m_resource_manager.release(m_id, m_pad_execute_client->result());
}


bool 
Request::check_cancel()
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

bool 
Request::owns_lock() const
{
    return m_executing;
}

bool 
Request::try_acquire()
{
    if (!m_executing && m_resource_manager.try_lock(m_id)) {
        m_executing = true;
        m_acquire_time = m_clock->now();
        RCLCPP_INFO(m_logger, "Acquired lock.");
        return true;
    }
    return false;
}

bool 
Request::hold_time_exceeded(const rclcpp::Duration & max_hold_time) const
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

void 
Request::publish_feedback(
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
    RCLCPP_INFO(m_logger, "Publishing feedback: status %i, time_remaining: %f", feedback->status, feedback->time_remaining);
}