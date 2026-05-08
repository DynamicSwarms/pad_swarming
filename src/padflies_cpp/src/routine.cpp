#include "padflies_cpp/routine.hpp"



Routine::Routine(
    BT::Tree&& routine,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface, 
    rclcpp::Logger logger
  )
    : m_logger(logger.get_child("RoutineExecutor"))
    , m_behavior_tree(std::move(routine))
  {
    m_timer = rclcpp::create_timer(
        node_base_interface,
        node_timers_interface,
        node_clock_interface->get_clock(),
        std::chrono::milliseconds(100),
        std::bind(&Routine::m_timer_callback, this));
  }

  void 
  Routine::halt() 
  {
    m_tree_is_running = false;
    m_behavior_tree.haltTree();
    if (m_on_finished_callback) {
      m_on_finished_callback(false); // Consider halting as a failure
    }
  }

  void 
  Routine::m_timer_callback()
  {
    if (m_tree_is_running) {
      BT::NodeStatus status = m_behavior_tree.tickOnce();
      if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE || status == BT::NodeStatus::SKIPPED) {
          m_tree_is_running = false;
          RCLCPP_INFO(m_logger, "Behavior tree finished with status: %s", toStr(status).c_str());
          if (m_on_finished_callback) {
            m_on_finished_callback(status == BT::NodeStatus::SUCCESS);
          }
      } else{
          // RCLCPP_INFO(m_logger, "Behavior tree ticked with status: %s", toStr(status).c_str());
      }
    }    
  }

  void 
  Routine::start()
  {
    m_tree_is_running = true;
  }

  bool 
  Routine::is_running()
  {
    return m_tree_is_running;
  }

