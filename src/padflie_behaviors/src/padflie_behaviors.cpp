#include "padflie_behaviors/padflie_behaviors.hpp"
#include "padflie_behaviors/behaviors.hpp"
#include "padflie_behaviors/pad_client_factory.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace padflie_behaviors
{
static RoutineResult classify_tree(
  const BT::Tree & tree, const std::shared_ptr<FailureContext> & failure_context)
{

  if (const auto result = failure_context->result()) {
    return *result;
  }
  return {RoutineOutcome::SUCCESS, RoutineFailureReason::NONE, {}};
}

static std::pair<std::shared_ptr<PadExecuteServer>, std::shared_ptr<PadClientFactory>>
create_pad_interfaces(
  const padflies_cpp::NodeInterfacesBundle & interfaces,
  const std::shared_ptr<PadflieTF> & padflie_tf,
  const rclcpp::Logger & logger)
{
  const std::string prefix = interfaces.base_interface->get_name();
  auto server = std::make_shared<PadExecuteServer>(
    prefix, interfaces.base_interface, interfaces.clock_interface,
    interfaces.logging_interface, interfaces.waitables_interface,
    interfaces.base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive));
  auto factory = std::make_shared<PadClientFactory>(
    prefix, padflie_tf, interfaces.base_interface, interfaces.graph_interface,
    interfaces.logging_interface, interfaces.waitables_interface,
    interfaces.services_interface,
    interfaces.base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
    logger);
  return {std::move(server), std::move(factory)};
}

static BT::Tree create_tree(
  BT::BehaviorTreeFactory & factory, const std::string & tree_id,
  const padflies_cpp::NodeInterfacesBundle & node_interfaces_bundle,
  rclcpp::Logger logger, std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf, std::shared_ptr<PadExecuteServer> server,
  std::shared_ptr<PadClient> pad_client,
  std::shared_ptr<FailureContext> failure_context)
{
  factory.registerNodeType<GetPadRight>(
    "GetPadRight", logger, server, failure_context);
  factory.registerNodeType<HoldPadRight>(
    "HoldPadRight", logger, server, failure_context);
  factory.registerNodeType<TakeoffInit>("TakeoffInit", logger, failure_context);
  factory.registerNodeType<TakeoffRoutine>(
    "Takeoff", logger, node_interfaces_bundle.clock_interface, hardware_actor, padflie_tf, server,
    failure_context);
  factory.registerNodeType<LandRoutine>(
    "Land", logger, node_interfaces_bundle.clock_interface, hardware_actor, padflie_tf, server,
    failure_context);
  factory.registerNodeType<LandInit>("LandInit", logger, failure_context);
  factory.registerNodeType<ApproachIDLE>(
    "ApproachIDLE", logger, hardware_actor, padflie_tf, server, failure_context);
  factory.registerNodeType<ApproachCLOSE>(
    "ApproachCLOSE", logger, hardware_actor, padflie_tf, server, failure_context);
  factory.registerNodeType<TimeoutROS>(
    "TimeoutROS", logger, node_interfaces_bundle.clock_interface, failure_context);
  factory.registerNodeType<TryFinally>("TryFinally", logger, failure_context);
  factory.registerNodeType<SendFeedback>("SendFeedback", logger, server, failure_context);
  factory.registerNodeType<HasPadRight>("HasPadRight", logger, failure_context);
  factory.registerNodeType<ReleasePadRight>(
    "ReleasePadRight", logger, server, failure_context);
  const auto xml_path = ament_index_cpp::get_package_share_path("padflie_behaviors") /
    "config/behaviors.xml";
  factory.registerBehaviorTreeFromFile(xml_path.string());
  auto tree = factory.createTree(tree_id);
  tree.rootBlackboard()->set("client", std::move(pad_client));
  return tree;
}

BT::Tree PadflieTakeoffPlugin::getTree(
  BT::BehaviorTreeFactory & factory, 
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  const pad_management_interfaces::msg::SiteInfo & site_info)
{
  m_failure_context = std::make_shared<FailureContext>();
  auto [pad_execute_server, pad_client_factory] = create_pad_interfaces(
    m_node_interfaces_bundle, padflie_tf, m_logger);
  auto pad_client = pad_client_factory->create_pad_client(
    site_info.name, site_info.pad_right_control_action_name,
    site_info.pad_idle_target_service_name);
  return create_tree(
    factory, "TakeoffBehavior", m_node_interfaces_bundle, m_logger,
    hardware_actor, padflie_tf, pad_execute_server, pad_client, m_failure_context);
}

BT::Tree PadflieLandingPlugin::getTree(
  BT::BehaviorTreeFactory & factory, 
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  const pad_management_interfaces::msg::SiteInfo & site_info)
{
  m_failure_context = std::make_shared<FailureContext>();
  auto [pad_execute_server, pad_client_factory] = create_pad_interfaces(
    m_node_interfaces_bundle, padflie_tf, m_logger);
  auto pad_client = pad_client_factory->create_pad_client(
    site_info.name, site_info.pad_right_control_action_name,
    site_info.pad_idle_target_service_name);
  return create_tree(
    factory, "LandBehavior", m_node_interfaces_bundle, m_logger,
    hardware_actor, padflie_tf, pad_execute_server, pad_client, m_failure_context);
}

RoutineResultClassifier PadflieTakeoffPlugin::getResultClassifier() const
{
  const auto failure_context = m_failure_context;
  return [failure_context](const BT::Tree & tree) {
    return classify_tree(tree, failure_context);
  };
}

RoutineResultClassifier PadflieLandingPlugin::getResultClassifier() const
{
  const auto failure_context = m_failure_context;
  return [failure_context](const BT::Tree & tree) {
    return classify_tree(tree, failure_context);
  };
}
}  // namespace padflie_behaviors

PLUGINLIB_EXPORT_CLASS(padflie_behaviors::PadflieTakeoffPlugin, padflies_cpp::ITakeoffPlugin)
PLUGINLIB_EXPORT_CLASS(padflie_behaviors::PadflieLandingPlugin, padflies_cpp::ILandingPlugin)
