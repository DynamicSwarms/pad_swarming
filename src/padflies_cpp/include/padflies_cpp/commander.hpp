#include "padflies_cpp/commander_base.hpp"
#include "padflies_cpp/pad_control.hpp"
#include "padflies_cpp/routine.hpp"
#include "padflies_cpp/routine_factory.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include "padflies_cpp/command.hpp"

#include "padflies_cpp/command_context_interface.hpp"
#include "padflies_cpp/commander_state.hpp"

class PadflieCommander 
        : public PadflieCommanderBase,
          public ICommandContext
{
    public: 
        PadflieCommander(
            const std::string & prefix,
            const std::string & cf_prefix,
            std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface
        );

        void m_command_queue_execute();

        bool is_healthy() const override;

        bool get_home_state() const override;

        bool can_takeoff() const override;
        bool can_land() const override;
        bool is_flying() const override;

    private:
        
        void m_configure_commander(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        ) override;
        void m_on_commander_configured() override;

        void m_activate_commander(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        ) override;
        void m_on_commander_activated() override;

        void m_deactivate_commander(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
            bool force
        ) override;
        void m_on_commander_deactivated() override;

        void m_on_charged_callback() override;

    private:
        void m_handle_landing_target_timer();    
        void m_acquire_pad_right_callback(bool success);
    
    private: 
        void m_trigger_landing();
        
        void m_handle_takeoff_command(
            const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
            const std::shared_ptr<rmw_request_id_t> request_id,
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req
        ) override;

        void m_handle_land_command(
            const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
            const std::shared_ptr<rmw_request_id_t> request_id,
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req
        ) override;

        void m_on_takeoff_finished(bool success);
        void m_on_land_finished(bool success);

        void m_handle_send_target_command(
            const padflies_interfaces::msg::SendTarget::SharedPtr msg
        ) override;

    private: 
        void createBehaviorTree();


    private: 
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_node_base_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_node_timers_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_node_clock_interface;

        std::shared_ptr<PadControl> m_pad_control;
        std::shared_ptr<rclcpp::Clock> m_clock;

        std::shared_ptr<RoutineFactory> m_routine_factory;


        bool m_deactivating = false;
        bool m_commander_is_healthy = true;

        
        CommanderState m_state = CommanderState::UNCONFIGURED;


        std::shared_ptr<rclcpp::TimerBase> m_landing_target_timer;
        
        
        std::shared_ptr<Routine> m_routine;

        std::shared_ptr<rclcpp::TimerBase> m_command_queue_timer;
        std::mutex m_command_queue_mutex;
        std::queue<std::shared_ptr<Command>> m_command_queue;
        bool m_tree_is_running = false;
        BT::BehaviorTreeFactory m_bt_factory;
        BT::Tree m_behavior_tree;
        std::unique_ptr<BT::Groot2Publisher> m_bt_groot_publisher;

        std::chrono::steady_clock::time_point m_takeoff_command_time;
};