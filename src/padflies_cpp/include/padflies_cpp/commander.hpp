#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


#include "padflies_cpp/charge_controller.hpp"
#include "padflies_cpp/padflie_tf.hpp"
#include "padflies_cpp/actor.hpp"

class PadflieCommander{
    public: 
        PadflieCommander(
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

        void on_charged_callback();



    private: 
        std::string m_cf_prefix;

        ChargeController m_charge_controller;
        PadflieTF m_padflie_tf;

        std::unique_ptr<PadflieActor> m_padflie_actor;
    private: 
        uint8_t m_pad_id;    
};