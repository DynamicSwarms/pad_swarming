#pragma once
#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "padflies_cpp/routine.hpp"

#include "padflies_cpp/hardware_actor.hpp"
#include "padflies_cpp/pad_client_factory.hpp"
#include "padflies_cpp/pad_execute_server.hpp"
class RoutineFactory
{
public:
    RoutineFactory(
      std::shared_ptr<HardwareActor> hardware_actor,
      std::shared_ptr<PadflieTF> padflie_tf,
      std::shared_ptr<PadExecuteServer> pad_execute_server,
      std::shared_ptr<PadClientFactory> pad_client_factory,
      std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
      std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
      std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
      rclcpp::Logger logger,
      const std::string & xml_file = std::string("/home/winni/2ds/pad_swarming/install/padflies_cpp/share/padflies_cpp/behaviors/behaviors.xml")
    );
    ~RoutineFactory();

    std::shared_ptr<Routine> 
    create_routine(const std::string& tree_name);

private: 
    BT::BehaviorTreeFactory m_bt_factory;

    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_node_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_node_timers_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_node_clock_interface;
    rclcpp::Logger m_logger;
  };