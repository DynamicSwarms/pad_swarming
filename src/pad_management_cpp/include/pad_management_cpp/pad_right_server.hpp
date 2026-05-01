#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pad_management_interfaces/action/pad_right_control.hpp"

#include "pad_management_cpp/request_map.hpp"
#include "pad_management_cpp/pad_execute_client.hpp"
class PadRightServer
{
public:
  PadRightServer(
    IPadRightLock & pad_right_lock,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface
  );

private:
    void manage_requests();

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const pad_management_interfaces::action::PadRightControl::Goal> goal);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle);

    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle);


private: 
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_node_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_node_timers_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> m_node_graph_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_node_logging_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> m_node_waitables_interface;

    int m_max_requests;
    rclcpp::Duration m_max_hold_time;

    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
    rclcpp::Logger m_logger;
    std::unique_ptr<RequestMap> m_request_map;
    int m_requests_counter = 0;

    std::shared_ptr<rclcpp::TimerBase> m_execution_timer;
    std::shared_ptr<rclcpp_action::Server<pad_management_interfaces::action::PadRightControl>> m_action_server;

    std::shared_ptr<PadExecuteClient> m_pad_execute_client;
};