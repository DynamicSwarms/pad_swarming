#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "padflies_cpp/commander/command/routine/routine_result.hpp"

class Routine {
    
  public:
  Routine(
    BT::Tree&& routine,
    RoutineResultClassifier result_classifier,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface
  );


  void start();

  void halt();

  bool is_running();

  bool is_finished() const {
    return m_finished;
  };

  bool has_been_started() const {
    return m_has_been_started;
  }

  void set_on_finished_callback(std::function<void(RoutineResult)> callback)
  {
    m_on_finished_callback = std::move(callback);
  }
  
private: 
  void m_timer_callback();


private:
  RoutineResultClassifier m_result_classifier;
  BT::Tree m_behavior_tree;
  std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
  std::shared_ptr<rclcpp::TimerBase> m_timer;

  bool m_has_been_started = false;
  bool m_finished = false;
  bool m_tree_is_running = false;

  std::function<void(RoutineResult)> m_on_finished_callback;
  
};
