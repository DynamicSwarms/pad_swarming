#pragma once
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "padflies_cpp/commander/command/routine/routine_result.hpp"
#include "padflies_cpp/commander/command/routine/routine_interruption.hpp"

#include <mutex>

class Routine : public std::enable_shared_from_this<Routine> {
    
  public:
  Routine(
    BT::Tree&& routine,
    RoutineResultClassifier result_classifier,
    std::shared_ptr<RoutineInterruptionState> interruption,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface
  );


  void start();

  void halt();

  bool is_running();

  bool request_interruption_if_possible()
  {
    return m_interruption->request_interruption_if_allowed();
  }

  bool is_finished() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_finished;
  };

  bool has_been_started() const {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_has_been_started;
  }

  void set_on_finished_callback(std::function<void(RoutineResult)> callback)
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_on_finished_callback = std::move(callback);
  }
  
private: 
  void m_timer_callback();


private:
  RoutineResultClassifier m_result_classifier;
  std::shared_ptr<RoutineInterruptionState> m_interruption;
  BT::Tree m_behavior_tree;
  std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
  std::shared_ptr<rclcpp::TimerBase> m_timer;
  std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_node_base_interface;
  std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> m_node_timers_interface;
  std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_node_clock_interface;
  mutable std::mutex m_mutex;

  bool m_has_been_started = false;
  bool m_finished = false;
  bool m_tree_is_running = false;

  std::function<void(RoutineResult)> m_on_finished_callback;
  
};
