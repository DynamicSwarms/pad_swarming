#include "pad_management_cpp/pad_right_server.hpp"

PadRightServer::PadRightServer(
    IPadRightLock & pad_right_lock,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface)
:   m_node_base_interface(node_base_interface),
    m_node_timers_interface(node_timers_interface),
    m_node_logging_interface(node_logging_interface),
    m_node_graph_interface(node_graph_interface),
    m_node_waitables_interface(node_waitables_interface),
    m_max_requests(node_param_interface->declare_parameter(
        "max_requests", rclcpp::ParameterValue(10), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>()),
    m_max_hold_time(duration_from_seconds(node_param_interface->declare_parameter(
        "max_hold_time", rclcpp::ParameterValue(40.0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<double>())),
    m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)),
    m_logger(node_logging_interface->get_logger()),
    m_request_map(std::make_unique<RequestMap>(
        pad_right_lock,
        m_max_requests, 
        m_max_hold_time,
        node_clock_interface,
        node_logging_interface->get_logger().get_child("RequestServer")
    ))
{
    m_execution_timer = rclcpp::create_timer(
        node_base_interface,
        node_timers_interface,
        node_clock_interface->get_clock(),
        std::chrono::milliseconds(100),
        std::bind(&PadRightServer::manage_requests, this),
        m_callback_group
    );

    m_action_server = rclcpp_action::create_server<pad_management_interfaces::action::PadRightControl>(
        node_base_interface,
        node_clock_interface,
        node_logging_interface,
        node_waitables_interface,
        "~/pad_right_control",
        std::bind(&PadRightServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PadRightServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&PadRightServer::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(),
        m_callback_group
    );
}


void 
PadRightServer::manage_requests()
{
    if (m_request_map->manage_requests()) {
        // Do something if a new owner is selected
        //std::string current_holder = m_request_map->get_current_holder();
        //m_pad_execute_client = std::make_shared<PadExecuteClient>(
        //    current_holder,
        //    m_node_base_interface,
        //    m_node_graph_interface,
        //    m_node_logging_interface,
        //    m_node_waitables_interface,
        //    m_callback_group
        //);

        //m_pad_execute_client->send_goal(current_holder, pad_management_interfaces::action::PadExecute::Goal::ACTION_TAKEOFF); 

    }
}

rclcpp_action::GoalResponse PadRightServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const pad_management_interfaces::action::PadRightControl::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(m_logger, "Received goal request with name %s.", goal->name.c_str());

    if (m_request_map->has_name(goal->name)) {
      RCLCPP_INFO(m_logger, "Goal with name %s already exists, rejecting new goal.", goal->name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!m_request_map->fits_more_requests()) {
      RCLCPP_INFO(m_logger, "Request map is full, rejecting new goal with name %s.", goal->name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void PadRightServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle)
{
    m_request_map->add_request(goal_handle);
}

rclcpp_action::CancelResponse PadRightServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}




