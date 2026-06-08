#pragma once
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "padflies_cpp/routine.hpp"

#include "padflies_cpp/hardware_actor.hpp"
#include "padflies_cpp/pad_client_factory.hpp"
#include "padflies_cpp/pad_execute_server.hpp"

#include "pluginlib/class_loader.hpp"

#include "padflies_cpp/node_interfaces_bundle.hpp"
#include "padflies_cpp/I_padflie_behavior_plugin.hpp"

class RoutineFactory
{
public:
    RoutineFactory(
      padflies_cpp::NodeInterfacesBundle node_interfaces_bundle,
      rclcpp::Logger logger);
    ~RoutineFactory() = default;

    void set_padflie_shared_ptrs(
      std::shared_ptr<HardwareActor> hardware_actor,
      std::shared_ptr<PadflieTF> padflie_tf,
      std::shared_ptr<PadExecuteServer> pad_execute_server,
      std::shared_ptr<PadClientFactory> pad_client_factory
    );

    void reset_padflie_shared_ptrs();

    std::shared_ptr<Routine>
    create_takeoff_routine();

    std::shared_ptr<Routine>
    create_land_routine();


private: 
    void m_register_base_nodes(
      BT::BehaviorTreeFactory & factory,
      std::shared_ptr<HardwareActor> hardware_actor,
      std::shared_ptr<PadflieTF> padflie_tf,
      std::shared_ptr<PadExecuteServer> pad_execute_server,
      std::shared_ptr<PadClientFactory> pad_client_factory
    );

    rcl_interfaces::msg::SetParametersResult 
        m_set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters);

    void m_set_plugin();
private: 
    pluginlib::ClassLoader<padflies_cpp::IPadflieBehaviorPlugin> m_behavior_plugin_loader;
    std::shared_ptr<padflies_cpp::IPadflieBehaviorPlugin> p_plugin;

    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> m_param_callback_handle; 
    std::shared_ptr<rclcpp::TimerBase> m_change_plugin_timer;
    std::string m_plugin_name;

    padflies_cpp::NodeInterfacesBundle m_node_interfaces_bundle;
    rclcpp::Logger m_logger;

  private: 
    std::shared_ptr<HardwareActor> m_hardware_actor;
    std::shared_ptr<PadflieTF> m_padflie_tf;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;
    std::shared_ptr<PadClientFactory> m_pad_client_factory;    
};