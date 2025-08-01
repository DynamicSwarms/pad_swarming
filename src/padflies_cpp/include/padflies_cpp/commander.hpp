#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "padflies_cpp/charge_controller.hpp"
#include "padflies_cpp/padflie_tf.hpp"
#include "padflies_cpp/actor.hpp"
#include "padflies_cpp/pad_control.hpp"

#include "std_msgs/msg/empty.hpp"
#include "padflies_interfaces/msg/send_target.hpp"




class PadflieCommander{
    public: 
        PadflieCommander(
            const std::string & prefix,
            const std::string & cf_prefix,
            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_iface
        );

        bool on_configure(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        bool on_activate(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        bool on_deactivate(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        void m_on_charged_callback();
    
    private: 
        void m_create_subscriptions(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        void m_remove_subscriptions(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        void m_handle_landing_target_timer();

        void m_acquire_pad_right_callback(bool success);

        void m_handle_takeoff_command(
            const std_msgs::msg::Empty::SharedPtr msg
        );

        void m_handle_land_command(
            const std_msgs::msg::Empty::SharedPtr msg
        );

        void m_handle_send_target_command(
            const padflies_interfaces::msg::SendTarget::SharedPtr msg
        );


    private: 
        enum class PendingCommand { NONE, TAKEOFF, LAND };
        PendingCommand m_pending_command = PendingCommand::NONE;

        enum class CommanderState {
            UNCONFIGURED,
            CHARGING, 
            CHARGED,
            WAITING_FOR_TAKEOFF_RIGHTS,
            TAKEOFF,
            FLYING,
            WAITING_FOR_LAND_RIGHTS, 
            LANDING,
            LANDED
        };
        CommanderState m_state = CommanderState::UNCONFIGURED;

        std::string m_prefix;
        std::string m_cf_prefix;

        rclcpp::CallbackGroup::SharedPtr m_callback_group;

        rclcpp::TimerBase::SharedPtr m_landing_target_timer;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_takeoff_sub;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_land_sub;
        rclcpp::Subscription<padflies_interfaces::msg::SendTarget>::SharedPtr m_send_target_sub;

        ChargeController m_charge_controller;
        PadflieTF m_padflie_tf;
        PadControl m_pad_control;

        std::unique_ptr<PadflieActor> m_padflie_actor;
    private: 
        uint8_t m_pad_id;    
};