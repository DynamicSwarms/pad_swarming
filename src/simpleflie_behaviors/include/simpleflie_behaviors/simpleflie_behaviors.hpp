#pragma once

#include "padflies_cpp/commander/command/routine/I_padflie_behavior_plugin.hpp"

namespace simpleflie_behaviors
{
class SimpleTakeoffPlugin : public padflies_cpp::ITakeoffPlugin
{
public:
  SimpleTakeoffPlugin(padflies_cpp::NodeInterfacesBundle, rclcpp::Logger logger)
  : m_logger(logger.get_child("SimpleTakeoffPlugin")) {}
  BT::Tree getTree(BT::BehaviorTreeFactory &, std::shared_ptr<HardwareActor>,
    std::shared_ptr<PadflieTF>, const pad_management_interfaces::msg::SiteInfo &) override;
private:
  rclcpp::Logger m_logger;
};

class SimpleLandingPlugin : public padflies_cpp::ILandingPlugin
{
public:
  SimpleLandingPlugin(padflies_cpp::NodeInterfacesBundle, rclcpp::Logger logger)
  : m_logger(logger.get_child("SimpleLandingPlugin")) {}
  BT::Tree getTree(BT::BehaviorTreeFactory &, std::shared_ptr<HardwareActor>,
    std::shared_ptr<PadflieTF>, const pad_management_interfaces::msg::SiteInfo &) override;
private:
  rclcpp::Logger m_logger;
};
}  // namespace simpleflie_behaviors
