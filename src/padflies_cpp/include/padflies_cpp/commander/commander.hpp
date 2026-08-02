#include "padflies_cpp/commander/commander_base.hpp"
#include "padflies_cpp/commander/command/routine/routine.hpp"
#include "padflies_cpp/commander/command/routine/routine_factory.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include "padflies_cpp/commander/commander_state.hpp"
#include "padflies_cpp/commander/site/site_selector.hpp"
#include "padflies_cpp/commander/goal/flight_goal_manager.hpp"
#include "padflies_cpp/commander/goal/rclcpp_commander_event_sink.hpp"
#include "padflies_cpp/commander/goal/ros_goal_completion.hpp"
#include "padflies_cpp/commander/goal/routine_flight_goal_executor.hpp"

#include "padflies_cpp/node_interfaces_bundle.hpp"
#include "padflies_interfaces/msg/availability_info.hpp"
#include "padflies_interfaces/srv/deploy_to.hpp"
#include "padflies_interfaces/srv/return_to.hpp"

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
        void m_on_state_callback() override;

        void m_create_availability_interface(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);
        void m_remove_availability_interface();

        void m_create_goal_services(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);
        void m_remove_goal_services();

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

        void m_handle_deploy_to_goal(
            const std::shared_ptr<rclcpp::Service<padflies_interfaces::srv::DeployTo>> service,
            const std::shared_ptr<rmw_request_id_t> request_id,
            const std::shared_ptr<padflies_interfaces::srv::DeployTo::Request> request);

        void m_handle_return_to_goal(
            const std::shared_ptr<rclcpp::Service<padflies_interfaces::srv::ReturnTo>> service,
            const std::shared_ptr<rmw_request_id_t> request_id,
            const std::shared_ptr<padflies_interfaces::srv::ReturnTo::Request> request);

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

        std::shared_ptr<rclcpp::Service<padflies_interfaces::srv::DeployTo>>
            m_deploy_to_service;
        std::shared_ptr<rclcpp::Service<padflies_interfaces::srv::ReturnTo>>
            m_return_to_service;
};
