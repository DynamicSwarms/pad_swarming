#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>

#include "rclcpp_action/server_goal_handle.hpp"

#include "padflies_cpp/commander/goal/completion/goal_completion.hpp"

namespace padflies_cpp::commander
{

class IActionGoalCompletion : public IGoalCompletion
{
public:
  virtual void set_cancel_requested(bool requested) = 0;
  virtual void finish_deferred_cancellation() = 0;
};

template<typename ActionT>
class ActionGoalCompletion final : public IActionGoalCompletion
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

  ActionGoalCompletion(
    std::shared_ptr<GoalHandle> goal_handle,
    std::function<void()> terminal_callback)
  : m_goal_handle(std::move(goal_handle)),
    m_terminal_callback(std::move(terminal_callback))
  {
    if (!m_goal_handle) {
      throw std::invalid_argument("Action goal completion requires a goal handle");
    }
  }

  void set_cancel_requested(bool requested) override
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_cancel_requested = requested;
  }

  void complete(const GoalResult & result) override
  {
    std::function<void()> terminal_callback;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      if (m_completed || m_deferred_cancellation) {
        throw std::logic_error("A flight goal action result was completed twice");
      }

      auto response = std::make_shared<typename ActionT::Result>();
      response->outcome = outcome_value(result.outcome);
      response->message = result.message;

      if (result.outcome == GoalOutcome::SUCCESS) {
        m_goal_handle->succeed(response);
      } else if (result.outcome == GoalOutcome::INTERRUPTED && m_cancel_requested) {
        if (!m_goal_handle->is_canceling()) {
          m_deferred_cancellation = result;
          return;
        }
        m_goal_handle->canceled(response);
      } else {
        m_goal_handle->abort(response);
      }
      m_completed = true;
      terminal_callback = m_terminal_callback;
    }
    if (terminal_callback) terminal_callback();
  }

  void finish_deferred_cancellation() override
  {
    std::function<void()> terminal_callback;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      if (!m_deferred_cancellation || !m_goal_handle->is_canceling()) return;

      auto response = std::make_shared<typename ActionT::Result>();
      response->outcome = outcome_value(m_deferred_cancellation->outcome);
      response->message = m_deferred_cancellation->message;
      m_goal_handle->canceled(response);
      m_deferred_cancellation.reset();
      m_completed = true;
      terminal_callback = m_terminal_callback;
    }
    if (terminal_callback) terminal_callback();
  }

private:
  static std::uint8_t outcome_value(GoalOutcome outcome)
  {
    switch (outcome) {
      case GoalOutcome::SUCCESS:
        return ActionT::Result::SUCCESS;
      case GoalOutcome::FAILURE:
        return ActionT::Result::FAILURE;
      case GoalOutcome::INTERRUPTED:
        return ActionT::Result::INTERRUPTED;
    }
    return ActionT::Result::FAILURE;
  }

  std::shared_ptr<GoalHandle> m_goal_handle;
  std::function<void()> m_terminal_callback;
  std::mutex m_mutex;
  bool m_cancel_requested{false};
  bool m_completed{false};
  std::optional<GoalResult> m_deferred_cancellation;
};

}  // namespace padflies_cpp::commander
