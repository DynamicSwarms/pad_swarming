#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "pad_management_interfaces/action/pad_right_control.hpp"
#include "pad_management_interfaces/msg/pad_info.hpp"

#include "pad_management_cpp/request_map.hpp"
#include "pad_management_cpp/pad_execute_client.hpp"
class PadRightServer
{
public:
  PadRightServer(
    const std::string & name,
    IPadResourceManager & pad_resource_manager,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface
  );

private:
    void publish_info(bool available);

    void manage_requests();

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const pad_management_interfaces::action::PadRightControl::Goal> goal);

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle);

    
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle);


private: 
    IPadResourceManager & m_pad_resource_manager;
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
    std::string m_action_server_name = "";
    std::shared_ptr<rclcpp_action::Server<pad_management_interfaces::action::PadRightControl>> m_action_server;

    std::shared_ptr<rclcpp::Publisher<pad_management_interfaces::msg::PadInfo>> m_pad_info_publisher;
    std::shared_ptr<rclcpp::TimerBase> m_info_publish_timer;

    std::unordered_map<std::string, std::shared_ptr<PadExecuteClient>> m_queued_clients;
};

inline PadRightServer::PadRightServer(
    const std::string & name,
    IPadResourceManager & pad_resource_manager,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface)
    : m_pad_resource_manager(pad_resource_manager)
    , m_node_base_interface(node_base_interface)
    , m_node_timers_interface(node_timers_interface)
    , m_node_graph_interface(node_graph_interface)
    , m_node_logging_interface(node_logging_interface)
    , m_node_waitables_interface(node_waitables_interface)
    , m_max_requests(node_param_interface->declare_parameter(
        "max_requests", rclcpp::ParameterValue(10), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , m_max_hold_time(duration_from_seconds(node_param_interface->declare_parameter(
        "max_hold_time", rclcpp::ParameterValue(40.0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<double>()))
    , m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , m_logger(node_logging_interface->get_logger())
    , m_request_map(std::make_unique<RequestMap>(
        pad_resource_manager,
        m_max_requests,
        m_max_hold_time,
        node_clock_interface,
        node_logging_interface->get_logger().get_child("RequestServer")))
{
    m_execution_timer = rclcpp::create_timer(
        node_base_interface,
        node_timers_interface,
        node_clock_interface->get_clock(),
        std::chrono::milliseconds(100),
        std::bind(&PadRightServer::manage_requests, this),
        m_callback_group
    );

    m_action_server_name = name + "/pad_right_control";
    m_action_server = rclcpp_action::create_server<pad_management_interfaces::action::PadRightControl>(
        node_base_interface,
        node_clock_interface,
        node_logging_interface,
        node_waitables_interface,
        m_action_server_name,
        std::bind(&PadRightServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PadRightServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&PadRightServer::handle_accepted, this, std::placeholders::_1),
        rcl_action_server_get_default_options(),
        m_callback_group
    );

    auto pub_options = rclcpp::PublisherOptions();
    pub_options.callback_group = m_callback_group;
    m_pad_info_publisher = rclcpp::create_publisher<pad_management_interfaces::msg::PadInfo>(
        node_topics_interface,
        "pad_management/pad_info",
        rclcpp::QoS(10).reliable().transient_local(),
        pub_options
    );

    m_pad_resource_manager.set_on_change_callback(std::bind(&PadRightServer::publish_info, this, std::placeholders::_1));
    publish_info(true);
}

inline void PadRightServer::publish_info(bool available)
{
    pad_management_interfaces::msg::PadInfo msg;
    msg.node_name = m_node_base_interface->get_name();
    msg.pad_right_control_action_name = m_action_server_name;
    msg.pad_tf_names = m_pad_resource_manager.get_pad_tf_names();
    msg.available = available;
    m_pad_info_publisher->publish(msg);
}

inline void PadRightServer::manage_requests()
{
    m_request_map->manage_requests();
}

inline rclcpp_action::GoalResponse PadRightServer::handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const pad_management_interfaces::action::PadRightControl::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(m_logger, "Received goal request with name %s.", goal->name.c_str());

    const std::string name = goal->name;
    try {
        std::stoi(name.substr(7));
    } catch (const std::exception & e) {
        (void)e;
        RCLCPP_WARN(m_logger, "Failed to extract ID from goal name: %s", goal->name.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (m_request_map->has_name(goal->name)) {
      RCLCPP_INFO(m_logger, "Goal with name %s already exists, rejecting new goal.", goal->name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!m_request_map->fits_more_requests()) {
      RCLCPP_INFO(m_logger, "Request map is full, rejecting new goal with name %s.", goal->name.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }

    m_queued_clients[name] = std::make_shared<PadExecuteClient>(
        name,
        m_node_base_interface,
        m_node_graph_interface,
        m_node_logging_interface,
        m_node_waitables_interface,
        m_callback_group
    );

    if (!m_queued_clients[name]->wait_for_action_server_available()) {
        RCLCPP_ERROR(m_logger, "PadExecute action server not available, rejecting goal with name %s.", goal->name.c_str());
        m_queued_clients.erase(name);
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

inline void PadRightServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle)
{
    std::string name = goal_handle->get_goal()->name;
    uint8_t action = goal_handle->get_goal()->action;

    bool success = m_queued_clients[name]->send_goal(name, action);
    if (!success) {
        auto result = std::make_shared<pad_management_interfaces::action::PadRightControl::Result>();
        result->success = false;
        result->reason = "Failed to send goal to PadExecute action server";
        goal_handle->abort(result);
        m_queued_clients.erase(name);
        RCLCPP_ERROR(m_logger, "Failed to send goal to PadExecute action server, aborting request.");
        return;
    }

    m_request_map->add_request(goal_handle, m_queued_clients[name]);
    m_queued_clients.erase(name);
}

inline rclcpp_action::CancelResponse PadRightServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<pad_management_interfaces::action::PadRightControl>> goal_handle)
{
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}
