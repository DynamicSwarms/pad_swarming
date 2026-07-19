#include "simpleflie_behaviors/simpleflie_behaviors.hpp"
using namespace std::chrono_literals;
namespace simpleflie_behaviors
{

namespace
{
RoutineResult classify_tree(const BT::Tree & tree)
{
  if (tree.rootNode()->status() == BT::NodeStatus::SUCCESS) {
    return {RoutineOutcome::SUCCESS, RoutineFailureReason::NONE, {}};
  }
  return {
    RoutineOutcome::FAILURE, RoutineFailureReason::NONE,
    "Simple behavior tree failed"};
}
}  // namespace

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
    std::this_thread::sleep_for(1s);
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
    std::this_thread::sleep_for(1s);
    return BT::NodeStatus::SUCCESS;
  }

private: 
  rclcpp::Logger m_logger;
  std::shared_ptr<HardwareActor> m_hardware_actor;
};


BT::Tree 
SimpleTakeoffPlugin::getTree(
  BT::BehaviorTreeFactory & factory,
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF>,
  const pad_management_interfaces::msg::SiteInfo &)
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
}


BT::Tree 
SimpleLandingPlugin::getTree(
  BT::BehaviorTreeFactory & factory,
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF>,
  const pad_management_interfaces::msg::SiteInfo &)
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
}

RoutineResultClassifier SimpleTakeoffPlugin::getResultClassifier() const
{
  return classify_tree;
}

RoutineResultClassifier SimpleLandingPlugin::getResultClassifier() const
{
  return classify_tree;
}


}  // namespace simpleflie_behaviors


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(simpleflie_behaviors::SimpleTakeoffPlugin, padflies_cpp::ITakeoffPlugin)
PLUGINLIB_EXPORT_CLASS(simpleflie_behaviors::SimpleLandingPlugin, padflies_cpp::ILandingPlugin)
