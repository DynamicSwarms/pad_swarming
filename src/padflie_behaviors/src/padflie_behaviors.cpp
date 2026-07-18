#include "padflie_behaviors/padflie_behaviors.hpp"
#include "padflie_behaviors/behaviors.hpp"
#include "padflie_behaviors/pad_client_factory.hpp"

#include <ament_index_cpp/get_package_share_path.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "pad_management_interfaces/action/pad_execute.hpp"

namespace padflie_behaviors
{
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

class ReleasePadRight : public BT::SyncActionNode
{
public:
  ReleasePadRight(
    const std::string & name, const BT::NodeConfig & config,
    rclcpp::Logger logger, std::shared_ptr<PadExecuteServer> server)
  : BT::SyncActionNode(name, config),
    m_logger(logger.get_child(name)),
    m_pad_execute_server(std::move(server)) {}

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<std::shared_ptr<PadClient>>("pad_client"), BT::InputPort<uint8_t>("status")};
  }

  BT::NodeStatus tick() override
  {
    uint8_t status;
    if (!getInput<std::shared_ptr<PadClient>>("pad_client") || !getInput("status", status)) {
      RCLCPP_ERROR(m_logger, "Missing pad_client or status input");
      return BT::NodeStatus::FAILURE;
    }

    using Feedback = pad_management_interfaces::action::PadExecute::Feedback;
    using Result = pad_management_interfaces::action::PadExecute::Result;
    if (status == Feedback::STATUS_LANDED) {
      m_pad_execute_server->send_result(Result::RESULT_ON_PAD);
    } else if (status == Feedback::STATUS_TAKEOFF_CLEARED_PAD) {
      m_pad_execute_server->send_result(Result::RESULT_NOT_ON_PAD);
    } else if (status >= Feedback::STATUS_LANDING_INIT &&
      status <= Feedback::STATUS_LANDING_APPROACH_CLOSE)
    {
      m_pad_execute_server->send_result(Result::RESULT_NOT_ON_PAD);
    } else {
      m_pad_execute_server->send_result(Result::RESULT_FAILURE);
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Logger m_logger;
  std::shared_ptr<PadExecuteServer> m_pad_execute_server;
};

static BT::Tree create_tree(
  BT::BehaviorTreeFactory & factory, const std::string & tree_id,
  const padflies_cpp::NodeInterfacesBundle & node_interfaces_bundle,
  rclcpp::Logger logger, std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf, std::shared_ptr<PadExecuteServer> server,
  std::shared_ptr<PadClient> pad_client)
{
  factory.registerNodeType<GetPadRight>("GetPadRight", logger, server);
  factory.registerNodeType<HoldPadRight>("HoldPadRight", logger, server);
  factory.registerNodeType<TakeoffInit>("TakeoffInit", logger);
  factory.registerNodeType<TakeoffRoutine>(
    "Takeoff", logger, node_interfaces_bundle.clock_interface, hardware_actor, padflie_tf, server);
  factory.registerNodeType<LandRoutine>(
    "Land", logger, node_interfaces_bundle.clock_interface, hardware_actor, padflie_tf, server);
  factory.registerNodeType<LandInit>("LandInit", logger);
  factory.registerNodeType<ApproachIDLE>(
    "ApproachIDLE", logger, hardware_actor, padflie_tf, server);
  factory.registerNodeType<ApproachCLOSE>(
    "ApproachCLOSE", logger, hardware_actor, padflie_tf, server);
  factory.registerNodeType<TimeoutROS>(
    "TimeoutROS", logger, node_interfaces_bundle.clock_interface);
  factory.registerNodeType<TryFinally>("TryFinally", logger);
  factory.registerNodeType<SendFeedback>("SendFeedback", logger, server);
  factory.registerNodeType<HasPadRight>("HasPadRight", logger);
  factory.registerNodeType<ReleasePadRight>("ReleasePadRight", logger, server);
  const auto xml_path = ament_index_cpp::get_package_share_path("padflie_behaviors") /
    "config/behaviors.xml";
  factory.registerBehaviorTreeFromFile(xml_path.string());
  auto tree = factory.createTree(tree_id);
  tree.rootBlackboard()->set("client", std::move(pad_client));
  return tree;
}

BT::Tree PadflieTakeoffPlugin::getTree(
  BT::BehaviorTreeFactory & factory, std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  const pad_management_interfaces::msg::SiteInfo & site_info)
{
  auto [pad_execute_server, pad_client_factory] = create_pad_interfaces(
    m_node_interfaces_bundle, padflie_tf, m_logger);
  auto pad_client = pad_client_factory->create_pad_client(
    site_info.name, site_info.pad_right_control_action_name,
    site_info.pad_idle_target_service_name);
  return create_tree(
    factory, "TakeoffBehavior", m_node_interfaces_bundle, m_logger,
    hardware_actor, padflie_tf, pad_execute_server, pad_client);
}

BT::Tree PadflieLandingPlugin::getTree(
  BT::BehaviorTreeFactory & factory, std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  const pad_management_interfaces::msg::SiteInfo & site_info)
{
  auto [pad_execute_server, pad_client_factory] = create_pad_interfaces(
    m_node_interfaces_bundle, padflie_tf, m_logger);
  auto pad_client = pad_client_factory->create_pad_client(
    site_info.name, site_info.pad_right_control_action_name,
    site_info.pad_idle_target_service_name);
  return create_tree(
    factory, "LandBehavior", m_node_interfaces_bundle, m_logger,
    hardware_actor, padflie_tf, pad_execute_server, pad_client);
}
}  // namespace padflie_behaviors

PLUGINLIB_EXPORT_CLASS(padflie_behaviors::PadflieTakeoffPlugin, padflies_cpp::ITakeoffPlugin)
PLUGINLIB_EXPORT_CLASS(padflie_behaviors::PadflieLandingPlugin, padflies_cpp::ILandingPlugin)
