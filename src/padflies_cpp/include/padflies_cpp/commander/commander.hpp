#include "padflies_cpp/commander/commander_base.hpp"
#include "padflies_cpp/commander/command/routine/routine.hpp"
#include "padflies_cpp/commander/command/routine/routine_factory.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include <behaviortree_cpp/loggers/groot2_publisher.h>

#include "padflies_cpp/commander/command/command.hpp"

#include "padflies_cpp/commander/command/command_context_interface.hpp"
#include "padflies_cpp/commander/commander_state.hpp"
#include "padflies_cpp/commander/site/site_selector.hpp"

#include "padflies_cpp/node_interfaces_bundle.hpp"
#include "padflies_interfaces/msg/availability_info.hpp"

class PadflieCommander 
        : public PadflieCommanderBase,
          public ICommandContext
{
    public: 
        PadflieCommander(
            const std::string & prefix,
            const std::string & cf_prefix,
            padflies_cpp::NodeInterfacesBundle node_interfaces_bundle
        );

        void m_command_queue_execute();
        void m_command_queue_on_deactivate();

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
        void m_on_state_callback() override;

        void m_create_availability_interface(
            const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> & node);
        void m_remove_availability_interface();

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

    private: 
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_node_base_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_node_timers_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_node_clock_interface;


        std::shared_ptr<SiteSelector> m_site_selector;
        std::shared_ptr<rclcpp::Publisher<padflies_interfaces::msg::AvailabilityInfo>>
            m_availability_pub;
        

        std::shared_ptr<RoutineFactory> m_routine_factory;
        std::shared_ptr<rclcpp::Clock> m_clock;


        CommanderState m_state = CommanderState::UNCONFIGURED;


        std::shared_ptr<rclcpp::TimerBase> m_landing_target_timer;
        
        
        std::shared_ptr<rclcpp::TimerBase> m_command_queue_timer;
        std::mutex m_command_queue_mutex;
        std::queue<std::shared_ptr<Command>> m_command_queue;
        rclcpp::Time m_command_start_time;
};
