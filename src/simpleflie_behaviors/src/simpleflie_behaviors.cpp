#include "simpleflie_behaviors/simpleflie_behaviors.hpp"

namespace simpleflie_behaviors
{

class TakeoffSimple : public BT::SyncActionNode
{
public:
  TakeoffSimple(
    const std::string& name,
    const BT::NodeConfig& config,
    rclcpp::Logger logger,
    std::shared_ptr<HardwareActor> hardware_actor)
  : BT::SyncActionNode(name, config)
  , m_logger(logger.get_child(name))
  , m_hardware_actor(hardware_actor)
  {

  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    m_hardware_actor->takeoff(1.0, 0.0, 4.0);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Logger m_logger;
  std::shared_ptr<HardwareActor> m_hardware_actor;
};


class LandSimple : public BT::SyncActionNode
{
public:
  LandSimple(
    const std::string& name,
    const BT::NodeConfig& config,
    rclcpp::Logger logger,
    std::shared_ptr<HardwareActor> hardware_actor)
  : BT::SyncActionNode(name, config)
  , m_logger(logger.get_child(name))
  , m_hardware_actor(hardware_actor)
{
}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    m_hardware_actor->land(0.0, 0.0, 4.0);
    return BT::NodeStatus::SUCCESS;
  }

private: 
  rclcpp::Logger m_logger;
  std::shared_ptr<HardwareActor> m_hardware_actor;
};


BT::Tree 
SimpleflieBehaviors::getTakeoffTree(
  BT::BehaviorTreeFactory & factory,
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  std::shared_ptr<PadExecuteServer> pad_execute_server,
  std::shared_ptr<PadClientFactory> pad_client_factory)
{
  std::string takeoff_tree_xml = R"(
  <root main_tree_to_execute="TakeoffBehavior">
    <BehaviorTree ID="TakeoffBehavior">
      <TakeoffSimple/>
    </BehaviorTree>
  </root>
  )";
  factory.registerNodeType<TakeoffSimple>(
    "TakeoffSimple", m_logger, hardware_actor);
  factory.registerBehaviorTreeFromText(takeoff_tree_xml);
  return factory.createTree("TakeoffBehavior");
  RCLCPP_INFO(m_logger, "Registered TakeoffSimple behavior in SimpleflieBehaviors plugin");
}


BT::Tree 
SimpleflieBehaviors::getLandTree(
  BT::BehaviorTreeFactory & factory,
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  std::shared_ptr<PadExecuteServer> pad_execute_server,
  std::shared_ptr<PadClientFactory> pad_client_factory)
{
  std::string land_tree_xml = R"(
  <root main_tree_to_execute="LandBehavior">
    <BehaviorTree ID="LandBehavior">
      <LandSimple/>
    </BehaviorTree>
  </root>
  )";
  factory.registerNodeType<LandSimple>(
    "LandSimple", m_logger, hardware_actor);
  factory.registerBehaviorTreeFromText(land_tree_xml);
  return factory.createTree("LandBehavior");
  RCLCPP_INFO(m_logger, "Registered LandSimple behavior in SimpleflieBehaviors plugin");
}


}  // namespace simpleflie_behaviors


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(simpleflie_behaviors::SimpleflieBehaviors, padflies_cpp::IPadflieBehaviorPlugin)