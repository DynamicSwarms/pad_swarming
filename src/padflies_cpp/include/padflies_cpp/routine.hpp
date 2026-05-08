#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"

class Routine {
    
  public:
  Routine(
    BT::Tree&& routine,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface, 
    rclcpp::Logger logger
  );


  void start();

  void halt();

  bool is_running();

  void set_on_finished_callback(std::function<void(bool success)> callback) 
  {
    m_on_finished_callback = std::move(callback);
  }
  
private: 
  void m_timer_callback();


private:
  std::function<void(bool success)> m_on_finished_callback;


  bool m_tree_is_running = false;
  BT::Tree m_behavior_tree;

  std::shared_ptr<rclcpp::TimerBase> m_timer;
  rclcpp::Logger m_logger;
};