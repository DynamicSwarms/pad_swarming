#include "padflies_cpp/commander_base.hpp"
#include "padflies_cpp/pad_control.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/loggers/groot2_publisher.h>
class PadflieCommander : public PadflieCommanderBase {
    public: 
        PadflieCommander(
            const std::string & prefix,
            const std::string & cf_prefix,
            std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface
        );

        void m_tick_tree_timer();

        bool is_healthy() const override;

        bool get_home_state() const override;

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
        
        bool m_process_takeoff_command() override;
        bool m_process_land_command() override;

        void m_handle_send_target_command(
            const padflies_interfaces::msg::SendTarget::SharedPtr msg
        ) override;

    private: 
        void createBehaviorTree();


    private: 
        PadControl m_pad_control;
        std::shared_ptr<rclcpp::Clock> m_clock;


        bool m_deactivating = false;
        bool m_commander_is_healthy = true;

        enum class CommanderState {
            UNCONFIGURED,
            CONFIGURED,
            CHARGING, 
            CHARGED,
            WAITING_FOR_TAKEOFF_RIGHTS,
            TAKEOFF,
            FLYING,
            WAITING_FOR_LAND_RIGHTS, 
            LANDING, 
            READY_TO_DEACTIVATE,
            FORCE_DEACTIVATE_RIGHT_WAIT
        };
        CommanderState m_state = CommanderState::UNCONFIGURED;


        std::shared_ptr<rclcpp::TimerBase> m_landing_target_timer;
        
        
        std::shared_ptr<rclcpp::TimerBase> m_tree_ticker_timer;
        bool m_tree_is_running = false;
        BT::BehaviorTreeFactory m_bt_factory;
        BT::Tree m_behavior_tree;
        std::unique_ptr<BT::Groot2Publisher> m_bt_groot_publisher;

        std::chrono::steady_clock::time_point m_takeoff_command_time;
};