#include "padflie_behaviors/padflie_behaviors.hpp"

namespace padflie_behaviors
{

class ChoosePadDefault : public BT::SyncActionNode
{
public:
  ChoosePadDefault(
    const std::string& name,
    const BT::NodeConfig& config,
    rclcpp::Logger logger)
  : BT::SyncActionNode(name, config)
  , m_logger(logger.get_child(name))
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    RCLCPP_INFO(m_logger, "ChoosePadDefault ticked");
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Logger m_logger;
};


BT::Tree 
PadflieBehaviors::getTakeoffTree(
  BT::BehaviorTreeFactory & factory,
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  std::shared_ptr<PadExecuteServer> pad_execute_server,
  std::shared_ptr<PadClientFactory> pad_client_factory)
{
  factory.registerNodeType<ChoosePadDefault>(
    "ChoosePadDefault",
  m_logger);
  factory.registerBehaviorTreeFromFile("/home/winni/ds/pad_swarming/install/padflies_cpp/share/padflies_cpp/behaviors/behaviors.xml");
  return factory.createTree("TakeoffBehavior");
  RCLCPP_INFO(m_logger, "Registered ChoosePadDefault behavior in PadflieBehaviorsBase plugin and created Takeoff tree");
}


BT::Tree 
PadflieBehaviors::getLandTree(
  BT::BehaviorTreeFactory & factory,
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  std::shared_ptr<PadExecuteServer> pad_execute_server,
  std::shared_ptr<PadClientFactory> pad_client_factory)
{
  factory.registerNodeType<ChoosePadDefault>(
    "ChoosePadDefault",
  m_logger);
  factory.registerBehaviorTreeFromFile("/home/winni/ds/pad_swarming/install/padflies_cpp/share/padflies_cpp/behaviors/behaviors.xml");
  return factory.createTree("LandBehavior");
  RCLCPP_INFO(m_logger, "Registered ChoosePadDefault behavior in PadflieBehaviorsBase plugin and created Land tree");
}


}  // namespace padflie_behaviors


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(padflie_behaviors::PadflieBehaviors, padflies_cpp::IPadflieBehaviorPlugin)