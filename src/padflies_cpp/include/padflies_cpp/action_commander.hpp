#pragma once

#include "padflies_cpp/commander_base.hpp"

#include "pad_management_interfaces/action/pad_execute.hpp"
#include "padflies_cpp/pad_client.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
class ActionCommander : public PadflieCommanderBase
{
public: 
    
    using PadExecuteActionT = pad_management_interfaces::action::PadExecute;
    using PadExecuteGoalHandleT = rclcpp_action::ServerGoalHandle<PadExecuteActionT>;

    ActionCommander(
        const std::string & prefix,
        const std::string & cf_prefix,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface)
    : PadflieCommanderBase(prefix, cf_prefix, node_base_interface, node_param_interface, node_clock_interface, node_logging_interface)
    , m_prefix(prefix)
    , m_logger(node_logging_interface->get_logger())
    , m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , m_node_base_interface(node_base_interface)
    , m_node_graph_interface(node_graph_interface)
    , m_node_services_interface(node_services_interface)
    , m_node_logging_interface(node_logging_interface)
    , m_node_waitables_interface(node_waitables_interface)
    , m_node_clock_interface(node_clock_interface)
    , m_node_timers_interface(node_timers_interface)
    {
        m_pad_execute_action_server = rclcpp_action::create_server<PadExecuteActionT>(
                node_base_interface,
                node_clock_interface,
                node_logging_interface,
                node_waitables_interface,
                prefix + "/pad_execute",
                std::bind(&ActionCommander::handle_pad_execute_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&ActionCommander::handle_pad_execute_cancel, this, std::placeholders::_1),
                std::bind(&ActionCommander::handle_pad_execute_accepted, this, std::placeholders::_1)
            );
    }
    
    bool is_healthy() const override { return true; }
    bool get_home_state() const override { return false;} // We are never "home"

    rclcpp_action::GoalResponse handle_pad_execute_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PadExecuteActionT::Goal> goal)
    {
        RCLCPP_INFO(m_logger, "Received goal request with pad name %s and action %d", goal->pad_name.c_str(), goal->action);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse handle_pad_execute_cancel(
        const std::shared_ptr<PadExecuteGoalHandleT> goal_handle)
    {
        RCLCPP_INFO(m_logger, "Received request to cancel goal");
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void handle_pad_execute_accepted(
        const std::shared_ptr<PadExecuteGoalHandleT> goal_handle)
    {
        RCLCPP_INFO(m_logger, "Padflie accepts the pads goal...");
        m_current_pad_execute_goal_handle = goal_handle;     
    }


    void takeoff_control_loop()
    {   
        auto clt = m_pad_client;
        if (!clt) {
            RCLCPP_ERROR(m_logger, "PadClient not initialized in takeoff control loop");
            return;
        }


        if (m_takeoff_state == TakeoffState::WAITING_FOR_GOAL_RESPONSE) 
        {
            if (clt->goal_responded()) 
            {
                if (clt->goal_accepted()) 
                {
                    m_takeoff_state = TakeoffState::WAITING_FOR_PAD_RIGHT;
                } else 
                {
                    RCLCPP_ERROR(m_logger, "Pad right control goal rejected");
                    respond_to_takeoff_command(false);
                }            
            }            
        } else if (m_takeoff_state == TakeoffState::WAITING_FOR_PAD_RIGHT) {
            if (clt->has_right())
            {
                RCLCPP_INFO(m_logger, "Acquired right, taking off...");

                auto feedback = std::make_shared<PadExecuteActionT::Feedback>();
                feedback->status = PadExecuteActionT::Feedback::STATUS_TAKEOFF_IN_PAD;
                m_current_pad_execute_goal_handle->publish_feedback(feedback);
                m_takeoff_state = TakeoffState::TAKING_OFF_IN_PAD;
            }
        } else if (m_takeoff_state == TakeoffState::TAKING_OFF_IN_PAD) {
            auto feedback = std::make_shared<PadExecuteActionT::Feedback>();
            feedback->status = PadExecuteActionT::Feedback::STATUS_TAKEOFF_LEFT_PAD;
            m_current_pad_execute_goal_handle->publish_feedback(feedback);
            m_takeoff_state = TakeoffState::TAKING_OFF_LEFT_PAD;
        } else if (m_takeoff_state == TakeoffState::TAKING_OFF_LEFT_PAD) {
            m_hardware_actor->takeoff(0.5, 0.0, 2.0);
            auto result = std::make_shared<PadExecuteActionT::Result>();

            result->result = PadExecuteActionT::Result::RESULT_NOT_ON_PAD;
            m_current_pad_execute_goal_handle->succeed(result);  
            m_takeoff_state = TakeoffState::TAKING_OFF_CLEARED_PAD;
        } else if (m_takeoff_state == TakeoffState::TAKING_OFF_CLEARED_PAD)
        {
            if (clt->received_result())
            {
                if (clt->result_success()) 
                {
                    RCLCPP_INFO(m_logger, "Takeoff successful!");
                } else 
                {
                    RCLCPP_ERROR(m_logger, "Takeoff failed according to pad.");
                }
                m_takeoff_control_timer->cancel();
            }
            // We could add some timeout or monitoring here to check if takeoff was successful
        }


    }

    void m_handle_takeoff_command(
        const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
        const std::shared_ptr<rmw_request_id_t> request_id,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req) override
    {   
        m_current_takeoff_request_id = request_id;
        m_current_takeoff_service_handle = service_handle;
        m_pad_client = std::make_shared<PadClient>(
            m_prefix,
            "megapad",
            m_padflie_tf,
            m_node_base_interface,
            m_node_graph_interface,
            m_node_logging_interface,
            m_node_waitables_interface,
            m_node_services_interface,
            m_callback_group,
            m_logger);

        RCLCPP_INFO(m_logger, "Waiting for action server %s to be available...", "megapad");
        if (!m_pad_client->is_action_server_available(std::chrono::seconds(1))) 
        {
            RCLCPP_ERROR(m_logger, "Action server %s not available after waiting", "megapad");
            respond_to_takeoff_command(false);
            return;
        }

        m_pad_client->send_request(PadClient::PadRightControlActionT::Goal::ACTION_TAKEOFF);
        m_takeoff_control_timer = rclcpp::create_timer(
            m_node_base_interface,
            m_node_timers_interface,
            m_node_clock_interface->get_clock(),
            std::chrono::milliseconds(100),
            std::bind(&ActionCommander::takeoff_control_loop, this),
            m_callback_group
        );
        m_takeoff_state = TakeoffState::WAITING_FOR_GOAL_RESPONSE;

    }

    void respond_to_takeoff_command(bool success)
    {
        auto response = std_srvs::srv::Trigger::Response();
        response.success = success;
        response.message = "Takeoff command accepted";
        m_current_takeoff_service_handle->send_response(*m_current_takeoff_request_id, response);
    }


    void m_handle_land_command(
        const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
        const std::shared_ptr<rmw_request_id_t> request_id,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req) override
    {
        respond_to_land_command(service_handle, request_id);
    }

    void respond_to_land_command(
        const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
        const std::shared_ptr<rmw_request_id_t> request_id)
    {
        auto response = std_srvs::srv::Trigger::Response();
        response.success = true;
        response.message = "Land command accepted";
        service_handle->send_response(*request_id, response);
    }

    void m_handle_send_target_command(const padflies_interfaces::msg::SendTarget::SharedPtr msg) override
    {
        RCLCPP_INFO(m_logger, "ActionCommander rejecting send target command");
    }


private: 
    std::string m_prefix;
    rclcpp::Logger m_logger;

    std::shared_ptr<rclcpp_action::Server<PadExecuteActionT>> m_pad_execute_action_server;
    std::shared_ptr<PadClient> m_pad_client;
    std::shared_ptr<rclcpp::TimerBase> m_takeoff_control_timer;
    enum class TakeoffState {
        WAITING_FOR_GOAL_RESPONSE,
        WAITING_FOR_PAD_RIGHT,
        TAKING_OFF_IN_PAD, 
        TAKING_OFF_LEFT_PAD,
        TAKING_OFF_CLEARED_PAD,
    };
    TakeoffState m_takeoff_state = TakeoffState::WAITING_FOR_GOAL_RESPONSE;

    std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> m_current_takeoff_service_handle;
    std::shared_ptr<rmw_request_id_t> m_current_takeoff_request_id;


    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_node_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> m_node_graph_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> m_node_services_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> m_node_logging_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> m_node_waitables_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_node_clock_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_node_timers_interface;

    std::shared_ptr<PadExecuteGoalHandleT> m_current_pad_execute_goal_handle;
};