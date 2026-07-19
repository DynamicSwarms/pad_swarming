#pragma once

#include "padflies_cpp/commander/command/routine/I_padflie_behavior_plugin.hpp"

namespace padflie_behaviors
{
class FailureContext;

class PadflieTakeoffPlugin : public padflies_cpp::ITakeoffPlugin
{
public:
  PadflieTakeoffPlugin(padflies_cpp::NodeInterfacesBundle interfaces, rclcpp::Logger logger)
  : m_node_interfaces_bundle(std::move(interfaces)),
    m_logger(logger.get_child("PadflieTakeoffPlugin")) {}

  BT::Tree getTree(
    BT::BehaviorTreeFactory & factory,
    std::shared_ptr<HardwareActor> hardware_actor,
    std::shared_ptr<PadflieTF> padflie_tf,
    const pad_management_interfaces::msg::SiteInfo & site_info) override;

  RoutineResultClassifier getResultClassifier() const override;

private:
  padflies_cpp::NodeInterfacesBundle m_node_interfaces_bundle;
  rclcpp::Logger m_logger;
  std::shared_ptr<FailureContext> m_failure_context;
};

class PadflieLandingPlugin : public padflies_cpp::ILandingPlugin
{
public:
  PadflieLandingPlugin(padflies_cpp::NodeInterfacesBundle interfaces, rclcpp::Logger logger)
  : m_node_interfaces_bundle(std::move(interfaces)),
    m_logger(logger.get_child("PadflieLandingPlugin")) {}

  BT::Tree getTree(
    BT::BehaviorTreeFactory & factory,
    std::shared_ptr<HardwareActor> hardware_actor,
    std::shared_ptr<PadflieTF> padflie_tf,
    const pad_management_interfaces::msg::SiteInfo & site_info) override;

  RoutineResultClassifier getResultClassifier() const override;

private:
  padflies_cpp::NodeInterfacesBundle m_node_interfaces_bundle;
  rclcpp::Logger m_logger;
  std::shared_ptr<FailureContext> m_failure_context;
};
}  // namespace padflie_behaviors
