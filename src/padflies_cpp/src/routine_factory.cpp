
#include "padflies_cpp/routine_factory.hpp"

#include "behaviors/behaviors.cpp"


RoutineFactory::RoutineFactory(
    std::shared_ptr<HardwareActor> hardware_actor,
    std::shared_ptr<PadflieTF> padflie_tf,
    std::shared_ptr<PadExecuteServer> pad_execute_server,
    std::shared_ptr<PadClientFactory> pad_client_factory,
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
    m_bt_factory.registerNodeType<ChoosePad>("ChoosePad",logger,  pad_client_factory);
    m_bt_factory.registerNodeType<GetPadRight>("GetPadRight", logger, pad_execute_server);
    m_bt_factory.registerNodeType<HoldPadRight>("HoldPadRight", logger, pad_execute_server);
    m_bt_factory.registerNodeType<ReleasePadRight>("ReleasePadRight", logger, pad_execute_server);
    m_bt_factory.registerNodeType<TakeoffInit>("TakeoffInit", logger);
    m_bt_factory.registerNodeType<TakeoffRoutine>("Takeoff", logger, node_clock_interface, hardware_actor, padflie_tf, pad_execute_server);
    m_bt_factory.registerNodeType<LandRoutine>("Land", logger, node_clock_interface, hardware_actor,padflie_tf, pad_execute_server);
    m_bt_factory.registerNodeType<LandInit>("LandInit", logger);
    m_bt_factory.registerNodeType<ApproachIDLE>("ApproachIDLE", logger, hardware_actor, padflie_tf, pad_execute_server);
    m_bt_factory.registerNodeType<ApproachCLOSE>("ApproachCLOSE", logger, hardware_actor, padflie_tf,  pad_execute_server);
    m_bt_factory.registerNodeType<TimeoutROS>("TimeoutROS", logger, node_clock_interface);
    m_bt_factory.registerNodeType<SendFeedback>("SendFeedback", logger, pad_execute_server);
    m_bt_factory.registerNodeType<HasPadRight>("HasPadRight", logger);
    m_bt_factory.registerBehaviorTreeFromFile(xml_file);
  }

RoutineFactory::~RoutineFactory()
{
  RCLCPP_INFO(m_logger, "RoutineFactory destructor called.");
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

    