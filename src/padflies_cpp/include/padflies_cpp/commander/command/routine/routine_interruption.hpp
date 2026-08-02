#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp/decorator_node.h"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

class RoutineInterruptionState
{
public:
  explicit RoutineInterruptionState(rclcpp::Logger logger)
  : m_logger(std::move(logger))
  {}

  void allow_interruption()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_interruption_allowed) {
      RCLCPP_DEBUG(m_logger, "Routine entered an interruptible region");
    }
    m_interruption_allowed = true;
  }

  void prevent_interruption()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_interruption_allowed) {
      RCLCPP_DEBUG(m_logger, "Routine left its interruptible region");
    }
    m_interruption_allowed = false;
  }

  bool request_interruption_if_allowed()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_interruption_allowed) {
      RCLCPP_DEBUG(m_logger, "Routine interruption request rejected");
      return false;
    }
    m_interruption_requested = true;
    RCLCPP_DEBUG(m_logger, "Routine interruption request accepted");
    return true;
  }

  bool take_interruption_request()
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    const bool requested = m_interruption_requested;
    m_interruption_requested = false;
    if (requested) {
      RCLCPP_DEBUG(m_logger, "Routine is processing the accepted interruption");
    }
    return requested;
  }

private:
  std::mutex m_mutex;
  rclcpp::Logger m_logger;
  bool m_interruption_allowed{false};
  bool m_interruption_requested{false};
};

class Interruptible : public BT::DecoratorNode
{
public:
  Interruptible(
    const std::string & name,
    const BT::NodeConfig & config,
    std::shared_ptr<RoutineInterruptionState> interruption)
  : BT::DecoratorNode(name, config),
    m_interruption(std::move(interruption))
  {}

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus tick() override
  {
    m_interruption->allow_interruption();
    const auto child_status = child_node_->executeTick();
    if (isStatusCompleted(child_status)) {
      m_interruption->prevent_interruption();
      resetChild();
    }
    return child_status;
  }

  void halt() override
  {
    m_interruption->prevent_interruption();
    DecoratorNode::halt();
  }

private:
  std::shared_ptr<RoutineInterruptionState> m_interruption;
};
