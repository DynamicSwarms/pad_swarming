#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "padflies_cpp/charge_controller.hpp"
#include "padflies_cpp/padflie_tf.hpp"
#include "padflies_cpp/actor.hpp"
#include "padflies_cpp/pad_control.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "padflies_interfaces/msg/send_target.hpp"
#include "padflies_interfaces/msg/padflie_info.hpp"




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
        
        void m_handle_info_timer();
        void m_handle_landing_target_timer();

        void m_trigger_landing();
        void m_trigger_takeoff();

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
        bool m_deactivating = false;

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
            READY_TO_DEACTIVATE
        };
        CommanderState m_state = CommanderState::UNCONFIGURED;

        std::string m_prefix;
        std::string m_cf_prefix;

        rclcpp::CallbackGroup::SharedPtr m_callback_group;

        rclcpp::TimerBase::SharedPtr m_landing_target_timer;

        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_takeoff_sub;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr m_land_sub;
        rclcpp::Subscription<padflies_interfaces::msg::SendTarget>::SharedPtr m_send_target_sub;

        rclcpp::TimerBase::SharedPtr m_info_timer;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr m_availability_pub;
        rclcpp::Publisher<padflies_interfaces::msg::PadflieInfo>::SharedPtr m_padflie_info_pub;

        ChargeController m_charge_controller;
        PadflieTF m_padflie_tf;
        PadControl m_pad_control;

        std::unique_ptr<PadflieActor> m_padflie_actor;
    private: 
        uint8_t m_pad_id; 
        
        std::string m_logger_name;
};