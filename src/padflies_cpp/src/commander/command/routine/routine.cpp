#include "padflies_cpp/commander/command/routine/routine.hpp"



Routine::Routine(
    BT::Tree&& routine,
    RoutineResultClassifier result_classifier,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface
  )
    : m_result_classifier(std::move(result_classifier))
    , m_behavior_tree(std::move(routine))
    , m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
  {
    m_timer = rclcpp::create_timer(
        node_base_interface,
        node_timers_interface,
        node_clock_interface->get_clock(),
        std::chrono::milliseconds(100),
        std::bind(&Routine::m_timer_callback, this),
        m_callback_group
      );
  }

  void 
  Routine::halt() 
  {
    m_tree_is_running = false;
    m_behavior_tree.haltTree();
    if (m_on_finished_callback) {
      m_on_finished_callback(
        m_result_classifier(m_behavior_tree));
    }
    m_finished = true;
  }

  void 
  Routine::m_timer_callback()
  {
    if (m_tree_is_running) {
      BT::NodeStatus status = m_behavior_tree.tickOnce();
      if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE || status == BT::NodeStatus::SKIPPED) {
          m_tree_is_running = false;
          if (m_on_finished_callback) {
            m_on_finished_callback(m_result_classifier(m_behavior_tree));
          }
          m_finished = true;

      }
    }    
  }

  void 
  Routine::start()
  {
    m_has_been_started = true;
    m_tree_is_running = true;
  }

  bool 
  Routine::is_running()
  {
    return m_tree_is_running;
  }
