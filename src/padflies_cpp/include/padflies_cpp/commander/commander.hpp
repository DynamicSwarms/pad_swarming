#include "padflies_cpp/commander/commander_base.hpp"
#include "padflies_cpp/commander/command/routine/routine.hpp"
#include "padflies_cpp/commander/command/routine/routine_factory.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include "padflies_cpp/commander/commander_state.hpp"
#include "padflies_cpp/commander/site/site_selector.hpp"
#include "padflies_cpp/commander/goal/flight_goal_manager.hpp"
#include "padflies_cpp/commander/goal/rclcpp_commander_event_sink.hpp"
#include "padflies_cpp/commander/goal/completion/action_goal_completion.hpp"
#include "padflies_cpp/commander/goal/completion/blocking_goal_completion.hpp"
#include "padflies_cpp/commander/goal/completion/trigger_goal_completion.hpp"
#include "padflies_cpp/commander/goal/routine_flight_goal_executor.hpp"

#include "padflies_cpp/node_interfaces_bundle.hpp"
#include "padflies_interfaces/msg/availability_info.hpp"
#include "padflies_interfaces/action/deploy.hpp"
#include "padflies_interfaces/action/return.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <map>
#include <mutex>

class PadflieCommander : public PadflieCommanderBase
{
    public: 
        PadflieCommander(
            const std::string & prefix,
            const std::string & cf_prefix,
            padflies_cpp::NodeInterfacesBundle node_interfaces_bundle
        );
        ~PadflieCommander() override;

        bool is_healthy() const override;
        bool get_home_state() const override;

    private:
        
        void m_configure_commander() override;
        void m_on_commander_configured() override;

        void m_activate_commander() override;
        void m_on_commander_activated() override;

        void m_deactivate_commander(bool force) override;
        void m_on_commander_deactivated() override;
        void m_cleanup_commander() override;

        void m_on_charged_callback() override;
        void m_on_state_callback() override;

        void m_create_availability_interface();
        void m_remove_availability_interface();

        void m_create_goal_interfaces();
        void m_remove_goal_interfaces();

        void m_on_goal_started(
            padflies_cpp::commander::FlightGoalKind goal_kind);
        void m_on_goal_finished(
            padflies_cpp::commander::FlightGoalKind goal_kind,
            padflies_cpp::commander::GoalResult result);

    private: 
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

        void m_handle_send_target_command(
            const padflies_interfaces::msg::SendTarget::SharedPtr msg
        ) override;

        rclcpp_action::GoalResponse m_handle_deploy_action_goal(
            const rclcpp_action::GoalUUID & goal_id,
            std::shared_ptr<const padflies_interfaces::action::Deploy::Goal> goal);
        rclcpp_action::CancelResponse m_handle_deploy_action_cancel(
            std::shared_ptr<rclcpp_action::ServerGoalHandle<padflies_interfaces::action::Deploy>>
                goal_handle);
        void m_handle_deploy_action_accepted(
            std::shared_ptr<rclcpp_action::ServerGoalHandle<padflies_interfaces::action::Deploy>>
                goal_handle);

        rclcpp_action::GoalResponse m_handle_return_action_goal(
            const rclcpp_action::GoalUUID & goal_id,
            std::shared_ptr<const padflies_interfaces::action::Return::Goal> goal);
        rclcpp_action::CancelResponse m_handle_return_action_cancel(
            std::shared_ptr<rclcpp_action::ServerGoalHandle<padflies_interfaces::action::Return>>
                goal_handle);
        void m_handle_return_action_accepted(
            std::shared_ptr<rclcpp_action::ServerGoalHandle<padflies_interfaces::action::Return>>
                goal_handle);

        template<typename ActionT>
        rclcpp_action::CancelResponse m_cancel_action_goal(
            const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> & goal_handle);

        void m_remove_action_goal(const rclcpp_action::GoalUUID & goal_id);

    private: 
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_node_base_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_node_timers_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_node_clock_interface;


        std::shared_ptr<SiteSelector> m_site_selector;
        std::shared_ptr<rclcpp::Publisher<padflies_interfaces::msg::AvailabilityInfo>>
            m_availability_pub;
        

        std::shared_ptr<RoutineFactory> m_routine_factory;
        std::unique_ptr<padflies_cpp::commander::RoutineFlightGoalExecutor> m_goal_executor;
        std::unique_ptr<padflies_cpp::commander::FlightGoalManager> m_goal_manager;

        CommanderState m_state = CommanderState::UNCONFIGURED;

        std::shared_ptr<rclcpp_action::Server<padflies_interfaces::action::Deploy>>
            m_deploy_action_server;
        std::shared_ptr<rclcpp_action::Server<padflies_interfaces::action::Return>>
            m_return_action_server;

        struct ActionGoalRecord
        {
            std::uint64_t internal_id;
            std::shared_ptr<padflies_cpp::commander::IActionGoalCompletion> completion;
        };
        std::map<rclcpp_action::GoalUUID, ActionGoalRecord> m_action_goals;
        std::mutex m_action_goals_mutex;
        std::shared_ptr<rclcpp::TimerBase> m_cancel_completion_timer;
        bool m_accepting_goals{false};
};
