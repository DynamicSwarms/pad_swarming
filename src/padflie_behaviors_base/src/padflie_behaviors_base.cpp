#include "padflie_behaviors_base/padflie_behaviors_base.hpp"

namespace padflie_behaviors_base
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


void 
PadflieBehaviorsBase::registerNodes(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<ChoosePadDefault>(
    "ChoosePadDefault",
    m_logger);

  RCLCPP_INFO(m_logger, "Registered ChoosePadDefault behavior in PadflieBehaviorsBase plugin");


}

}  // namespace padflie_behaviors_base


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(padflie_behaviors_base::PadflieBehaviorsBase, padflies_cpp::IPadflieBehaviorPlugin)