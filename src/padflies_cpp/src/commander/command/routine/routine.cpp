#include "padflies_cpp/commander/command/routine/routine.hpp"



Routine::Routine(
    BT::Tree&& routine,
    RoutineResultClassifier result_classifier,
    std::shared_ptr<RoutineInterruptionState> interruption,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface
  )
    : m_result_classifier(std::move(result_classifier))
    , m_interruption(std::move(interruption))
    , m_behavior_tree(std::move(routine))
    , m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    , m_node_base_interface(std::move(node_base_interface))
    , m_node_timers_interface(std::move(node_timers_interface))
    , m_node_clock_interface(std::move(node_clock_interface))
  {}

  void 
  Routine::halt() 
  {
    std::function<void(RoutineResult)> finished_callback;
    RoutineResult result;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_tree_is_running = false;
      m_behavior_tree.haltTree();
      result = m_result_classifier(RoutineTermination::HALTED);
      finished_callback = m_on_finished_callback;
      m_finished = true;
    }
    if (finished_callback) finished_callback(std::move(result));
  }

  void 
  Routine::m_timer_callback()
  {
    std::function<void(RoutineResult)> finished_callback;
    RoutineResult result;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      if (!m_tree_is_running) return;

      if (m_interruption->take_interruption_request()) {
        m_tree_is_running = false;
        m_behavior_tree.haltTree();
        result = {
          RoutineOutcome::INTERRUPTED,
          RoutineFailureReason::NONE,
          "Routine was interrupted"};
        finished_callback = m_on_finished_callback;
        m_finished = true;
      } else {

        BT::NodeStatus status = m_behavior_tree.tickOnce();
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE || status == BT::NodeStatus::SKIPPED) {
          m_tree_is_running = false;
          const auto termination = status == BT::NodeStatus::SUCCESS ?
            RoutineTermination::SUCCESS :
            status == BT::NodeStatus::FAILURE ?
            RoutineTermination::FAILURE : RoutineTermination::SKIPPED;
          result = m_result_classifier(termination);
          finished_callback = m_on_finished_callback;
          m_finished = true;
        }
      }
    }
    if (finished_callback) finished_callback(std::move(result));
  }

  void 
  Routine::start()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_timer) {
      std::weak_ptr<Routine> weak_routine = shared_from_this();
      m_timer = rclcpp::create_timer(
        m_node_base_interface,
        m_node_timers_interface,
        m_node_clock_interface->get_clock(),
        std::chrono::milliseconds(100),
        [weak_routine]() {
          if (const auto routine = weak_routine.lock()) {
            routine->m_timer_callback();
          }
        },
        m_callback_group);
    }
    m_has_been_started = true;
    m_tree_is_running = true;
  }

  bool 
  Routine::is_running()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_tree_is_running;
  }
