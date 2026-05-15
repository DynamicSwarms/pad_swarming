
#include "padflies_cpp/routine_factory.hpp"

#include "behaviors/hardware_actor_behaviors.cpp"
#include "behaviors/eigen_behaviors.cpp"
#include "behaviors/timing_behaviors.cpp"
#include "behaviors/pad_behaviors.cpp"


RoutineFactory::RoutineFactory(
    std::shared_ptr<HardwareActor> hardware_actor,
    std::shared_ptr<PadControl> pad_control,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    rclcpp::Logger logger,
    const std::string & xml_file)
  : m_bt_factory()
  , m_node_base_interface(node_base_interface)
  , m_node_timers_interface(node_timers_interface)
  , m_node_clock_interface(node_clock_interface)
  , m_logger(logger)
  {
    m_bt_factory.registerNodeType<HLCommandGoTo>("HLCommandGoTo", hardware_actor, logger);
    m_bt_factory.registerNodeType<HLCommandLand>("HLCommandLand", hardware_actor, logger);
    m_bt_factory.registerNodeType<HLCommandTakeoff>("HLCommandTakeoff", hardware_actor, logger);
    m_bt_factory.registerNodeType<LLCommanderSendTarget>("LLCommanderSendTarget", hardware_actor, logger);
    m_bt_factory.registerNodeType<CalculateAbovePadTargetAction>("CalculateAbovePadTarget", logger);
    m_bt_factory.registerNodeType<WaitFor>("WaitFor", node_clock_interface, logger);

    m_bt_factory.registerNodeType<SplitPose>("SplitPose");
    m_bt_factory.registerNodeType<AcquirePadRight>("AcquirePadRight", pad_control);
    m_bt_factory.registerSimpleAction("PrintStuff", [&](BT::TreeNode& self){
      RCLCPP_INFO(rclcpp::get_logger("PrintStuff"), "Hello from PrintStuff node!");
      return BT::NodeStatus::SUCCESS;
    });
    m_bt_factory.registerSimpleAction("PrintStuff2", [&](BT::TreeNode& self){
      RCLCPP_INFO(rclcpp::get_logger("PrintStuff2"), "Hello from PrintStuff2 node!");
      return BT::NodeStatus::SUCCESS;
    });

    m_bt_factory.registerBehaviorTreeFromFile(xml_file);
  }

std::shared_ptr<Routine> 
RoutineFactory::create_routine(const std::string& tree_name)
{
  return std::make_shared<Routine>(
    m_bt_factory.createTree(tree_name), 
    m_node_base_interface, 
    m_node_timers_interface, 
    m_node_clock_interface, 
    m_logger);
}

    