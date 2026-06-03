#pragma once

#include "padflies_cpp/I_padflie_behavior_plugin.hpp"
#include "padflies_cpp/node_interfaces_bundle.hpp"

namespace simpleflie_behaviors
{

class SimpleflieBehaviors : public padflies_cpp::IPadflieBehaviorPlugin
{
public:
   SimpleflieBehaviors(padflies_cpp::NodeInterfacesBundle node_interfaces_bundle, rclcpp::Logger logger)
  : m_node_interfaces_bundle(node_interfaces_bundle)
  , m_logger(logger.get_child("SimpleflieBehaviors"))
  {
  }

  BT::Tree getTakeoffTree(BT::BehaviorTreeFactory & factory, 
      std::shared_ptr<HardwareActor> hardware_actor,
      std::shared_ptr<PadflieTF> padflie_tf,
      std::shared_ptr<PadExecuteServer> pad_execute_server,
      std::shared_ptr<PadClientFactory> pad_client_factory) override;
  BT::Tree getLandTree(BT::BehaviorTreeFactory & factory, 
      std::shared_ptr<HardwareActor> hardware_actor,
      std::shared_ptr<PadflieTF> padflie_tf,
      std::shared_ptr<PadExecuteServer> pad_execute_server,
      std::shared_ptr<PadClientFactory> pad_client_factory) override;
private: 
    padflies_cpp::NodeInterfacesBundle m_node_interfaces_bundle;
    rclcpp::Logger m_logger;

};

}  // namespace simpleflie_behaviors

