#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

#include "padflies_cpp/commander/command/routine/I_padflie_behavior_plugin.hpp"
#include "padflies_cpp/node_interfaces_bundle.hpp"
#include "padflies_cpp/commander/command/routine/routine.hpp"

class RoutineFactory
{
public:
  RoutineFactory(padflies_cpp::NodeInterfacesBundle node_interfaces_bundle, rclcpp::Logger logger);

  void set_padflie_shared_ptrs(
    std::shared_ptr<HardwareActor> hardware_actor,
    std::shared_ptr<PadflieTF> padflie_tf);
  void reset_padflie_shared_ptrs();

  std::shared_ptr<Routine> create_takeoff_routine(
    const pad_management_interfaces::msg::SiteInfo & site_info);
  std::shared_ptr<Routine> create_land_routine(
    const pad_management_interfaces::msg::SiteInfo & site_info);

private:
  std::shared_ptr<Routine> m_make_routine(BT::Tree && tree);

  pluginlib::ClassLoader<padflies_cpp::ITakeoffPlugin> m_takeoff_plugin_loader;
  pluginlib::ClassLoader<padflies_cpp::ILandingPlugin> m_landing_plugin_loader;
  padflies_cpp::NodeInterfacesBundle m_node_interfaces_bundle;
  rclcpp::Logger m_logger;
  std::shared_ptr<HardwareActor> m_hardware_actor;
  std::shared_ptr<PadflieTF> m_padflie_tf;
};
