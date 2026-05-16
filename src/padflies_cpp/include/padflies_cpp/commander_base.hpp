#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "padflies_cpp/hardware_state_controller.hpp"
#include "padflies_cpp/padflie_tf.hpp"
#include "padflies_cpp/hardware_actor.hpp"
#include "padflies_cpp/pad_control.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "padflies_interfaces/msg/send_target.hpp"
#include "padflies_interfaces/msg/padflie_info.hpp"

#include "std_srvs/srv/trigger.hpp"


class CommanderException : public std::runtime_error {
public:
    explicit CommanderException(const std::string& message)
        : std::runtime_error(message) {}
};

class PadflieCommanderBase{
    public: 
        PadflieCommanderBase(
            const std::string & prefix,
            const std::string & cf_prefix,
            std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
            std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface
        );

        virtual ~PadflieCommanderBase();


        void on_configure(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        void on_activate(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        void on_deactivate(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
            bool force
        );


        virtual bool is_healthy() const = 0;
    
    private: 
        /**
         * Gets called when padflie gets configured. (Hardware is detected)
         * hw_state_controller not yet connected
         * no tf available
         * availability is not yet sent
         */
        virtual void m_configure_commander(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        ) {};
        /**
         * Gets called when base commander finished configuration. 
         * hw_state_controller is connected and starts listening to hardware state
         * tf is available and starts listening to padflie tf changes
         * availability is sent according to hardware state
         */
        virtual void m_on_commander_configured() {};

        /**
         * Gets called when padflie gets activated.
         * Prechecks are complete 
         * hardware_actor is not yet available
         * control interface is not yet available -> no commands incomming
         * availabilility is still sent
         */
        virtual void m_activate_commander(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        ) {};
        /**
         * Gets called when base commander finished activation. 
         * hardware_actor is available
         * control interface is available -> commands can be sent to padflie
         */
        virtual void m_on_commander_activated() {};

        /**
         * Gets called when padflie gets deactivated. 
         * Before this is called the control interface gets destroyed 
         * -> no more commands are beeing sent to padflie. 
         * The hardware_actor and hardware_state controller are still available.
         */
        virtual void m_deactivate_commander(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
            bool force
        ) {};
        /*
         * Gets called when base commander finished deactivation. 
         * hardware_actor is no longer available
         * hardware_state_controller is reset
         */
        virtual void m_on_commander_deactivated() {};

        virtual void m_on_charged_callback() {};
    
        virtual void m_handle_takeoff_command(
            const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
            const std::shared_ptr<rmw_request_id_t> request_id,
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req
        ) = 0;

        virtual void m_handle_land_command(
            const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
            const std::shared_ptr<rmw_request_id_t> request_id,
            const std::shared_ptr<std_srvs::srv::Trigger::Request> req
        ) = 0;

        virtual bool get_home_state() const {return false;};
          
        virtual void m_handle_send_target_command(
            const padflies_interfaces::msg::SendTarget::SharedPtr msg
        ) = 0;
        
        
    private:
        void m_on_state_callback();

        void m_handle_info_timer();        

        void m_create_availability_interface(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        void m_remove_availability_interface(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        void m_create_control_interface (
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        void m_remove_control_interface(
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );    
    

        rcl_interfaces::msg::SetParametersResult 
        m_set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters);

      


    protected: 
        std::string m_prefix;
        std::string m_cf_prefix;

        HardwareStateController m_hw_state_controller;
        PadflieTF m_padflie_tf;
        std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;

    private:

        std::string m_initial_pad;        
        std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> m_param_callback_handle; 
    protected:
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_node_clock_interface;
        rclcpp::Logger m_logger;
    private: 
        std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> m_takeoff_service;
        std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> m_land_service;
        std::shared_ptr<rclcpp::Subscription<padflies_interfaces::msg::SendTarget>> m_send_target_sub;

        std::shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> m_availability_pub;

        std::shared_ptr<rclcpp::TimerBase> m_padflie_info_timer;
        std::shared_ptr<rclcpp::Publisher<padflies_interfaces::msg::PadflieInfo>> m_padflie_info_pub;   
    protected:
        std::shared_ptr<HardwareActor> m_hardware_actor;

    private: 
        enum class CommanderBaseState {
            UNCONFIGURED,
            CONFIGURED,
            ACTIVATED
        };
        CommanderBaseState m_base_state = CommanderBaseState::UNCONFIGURED;
  
};