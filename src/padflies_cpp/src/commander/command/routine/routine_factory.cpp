#include "padflies_cpp/commander/command/routine/routine_factory.hpp"
#include "padflies_cpp/commander/command/routine/routine_interruption.hpp"

#include <stdexcept>

RoutineFactory::RoutineFactory(
  padflies_cpp::NodeInterfacesBundle node_interfaces_bundle, rclcpp::Logger logger)
: m_takeoff_plugin_loader("padflies_cpp", "padflies_cpp::ITakeoffPlugin"),
  m_landing_plugin_loader("padflies_cpp", "padflies_cpp::ILandingPlugin"),
  m_node_interfaces_bundle(std::move(node_interfaces_bundle)),
  m_logger(logger.get_child("RoutineFactory"))
{}

void RoutineFactory::set_padflie_shared_ptrs(
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf)
{
  if (!hardware_actor || !padflie_tf) {
    throw std::invalid_argument("Null shared pointer provided to RoutineFactory");
  }
  m_hardware_actor = std::move(hardware_actor);
  m_padflie_tf = std::move(padflie_tf);
}

void RoutineFactory::reset_padflie_shared_ptrs()
{
  m_hardware_actor.reset();
  m_padflie_tf.reset();
}

std::shared_ptr<Routine> RoutineFactory::create_takeoff_routine(
  const pad_management_interfaces::msg::SiteInfo & site_info)
{
  BT::BehaviorTreeFactory factory;
  auto interruption = std::make_shared<RoutineInterruptionState>(
    m_logger.get_child("Interruption"));
  factory.registerNodeType<Interruptible>("Interruptible", interruption);
  factory.registerNodeType<Interruptible>("Interruptable", interruption);
  auto plugin = m_takeoff_plugin_loader.createSharedInstance(
    site_info.takeoff_plugin_name, m_node_interfaces_bundle, m_logger);
  RCLCPP_INFO(m_logger, "Loaded takeoff plugin '%s' for site '%s'",
    site_info.takeoff_plugin_name.c_str(), site_info.name.c_str());
  auto tree = plugin->getTree(factory, m_hardware_actor, m_padflie_tf, site_info);
  auto result_classifier = plugin->getResultClassifier();
  return std::make_shared<Routine>(
    std::move(tree), std::move(result_classifier), std::move(interruption),
    m_node_interfaces_bundle.base_interface,
    m_node_interfaces_bundle.timers_interface,
    m_node_interfaces_bundle.clock_interface);
}

std::shared_ptr<Routine> RoutineFactory::create_land_routine(
  const pad_management_interfaces::msg::SiteInfo & site_info)
{
  BT::BehaviorTreeFactory factory;
  auto interruption = std::make_shared<RoutineInterruptionState>(
    m_logger.get_child("Interruption"));
  factory.registerNodeType<Interruptible>("Interruptible", interruption);
  factory.registerNodeType<Interruptible>("Interruptable", interruption);
  auto plugin = m_landing_plugin_loader.createSharedInstance(
    site_info.landing_plugin_name, m_node_interfaces_bundle, m_logger);
  RCLCPP_INFO(m_logger, "Loaded landing plugin '%s' for site '%s'",
    site_info.landing_plugin_name.c_str(), site_info.name.c_str());
  auto tree = plugin->getTree(factory, m_hardware_actor, m_padflie_tf, site_info);
  auto result_classifier = plugin->getResultClassifier();
  return std::make_shared<Routine>(
    std::move(tree), std::move(result_classifier), std::move(interruption),
    m_node_interfaces_bundle.base_interface,
    m_node_interfaces_bundle.timers_interface,
    m_node_interfaces_bundle.clock_interface);
}
