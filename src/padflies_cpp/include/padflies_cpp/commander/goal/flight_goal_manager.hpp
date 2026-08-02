#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "padflies_cpp/commander/goal/commander_event.hpp"
#include "padflies_cpp/commander/goal/flight_goal_executor.hpp"
#include "padflies_cpp/commander/goal/flight_goal_request.hpp"

namespace padflies_cpp::commander
{

class FlightGoalManager
{
public:
  FlightGoalManager(
    IFlightGoalExecutor & executor,
    std::shared_ptr<ICommanderEventSink> event_log =
    std::make_shared<NullCommanderEventSink>())
  : m_executor(executor), m_event_log(std::move(event_log))
  {
    if (!m_event_log) {
      throw std::invalid_argument("FlightGoalManager event log must not be null");
    }
  }

  std::uint64_t request_goal(
    FlightGoal goal,
    std::shared_ptr<IGoalCompletion> completion,
    GoalRetryPolicy retry_policy,
    GoalPolicy policy = GoalPolicy::REPLACEABLE)
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    return handle_goal_request(create_goal_request(
      std::move(goal), std::move(completion), retry_policy, policy));
  }

  void active_routine_finished(GoalResult result)
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    if (!m_active_goal) {
      return;
    }

    finish_goal(*m_active_goal, std::move(result));
    m_active_goal.reset();
    m_interruption_in_progress = false;
    m_no_active_goal.notify_all();
    start_next_goal_if_present();
  }

  bool has_next_goal() const noexcept
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    return m_next_goal.has_value();
  }

  void wait_until_no_goal_is_active()
  {
    std::unique_lock<std::recursive_mutex> lock(m_mutex);
    m_no_active_goal.wait(lock, [this] {return !m_active_goal;});
  }

  void cancel_all_goals(std::string reason)
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

    m_executor.stop_active_routine_without_callback();
    if (m_active_goal) {
      finish_goal(*m_active_goal, {GoalOutcome::INTERRUPTED, reason});
      m_active_goal.reset();
    }
    if (m_next_goal) {
      finish_goal(*m_next_goal, {GoalOutcome::INTERRUPTED, reason});
      m_next_goal.reset();
    }
    for (auto & goal : m_deferred_cancelled_goals) {
      finish_goal(goal, {GoalOutcome::INTERRUPTED, reason});
    }
    m_deferred_cancelled_goals.clear();
    m_interruption_in_progress = false;
    m_no_active_goal.notify_all();
  }

  bool cancel_goal(std::uint64_t goal_id)
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);

    if (m_active_goal && m_active_goal->id() == goal_id) {
      if (!m_active_goal->can_be_replaced()) return false;
      const bool accepted = try_interrupt_active_routine();
      if (accepted) m_interruption_in_progress = true;
      return accepted;
    }

    if (m_next_goal && m_next_goal->id() == goal_id) {
      if (!m_next_goal->can_be_replaced()) return false;
      m_deferred_cancelled_goals.push_back(std::move(*m_next_goal));
      m_next_goal.reset();
      return true;
    }

    return false;
  }

  void complete_deferred_cancellations()
  {
    std::lock_guard<std::recursive_mutex> lock(m_mutex);
    for (auto & goal : m_deferred_cancelled_goals) {
      finish_goal(goal, {GoalOutcome::INTERRUPTED, "Flight goal was cancelled"});
    }
    m_deferred_cancelled_goals.clear();
  }

private:
  FlightGoalRequest create_goal_request(
    FlightGoal goal,
    std::shared_ptr<IGoalCompletion> completion,
    GoalRetryPolicy retry_policy,
    GoalPolicy policy)
  {
    return {
      m_next_goal_id++, std::move(goal), std::move(completion), retry_policy, policy};
  }

  std::uint64_t handle_goal_request(FlightGoalRequest request)
  {
    const auto request_id = request.id();
    log_goal_event(CommanderEventType::GOAL_REQUESTED, request);

    if (!m_active_goal) {
      activate_goal_and_start_routine(std::move(request));
    } else if (replacement_is_locked()) {
      finish_goal_as_failed(request, "The active flight goal is locked");
    } else {
      request_interruption_and_set_next_goal(std::move(request));
    }
    return request_id;
  }

  void request_interruption_and_set_next_goal(FlightGoalRequest request)
  {
    if (!m_interruption_in_progress) {
      m_interruption_in_progress = try_interrupt_active_routine();
    }
    set_next_goal(std::move(request));
  }

  bool replacement_is_locked() const
  {
    return
      !m_active_goal->can_be_replaced() ||
      (m_next_goal && !m_next_goal->can_be_replaced());
  }

  bool try_interrupt_active_routine()
  {
    log_goal_event(CommanderEventType::INTERRUPTION_REQUESTED, *m_active_goal);
    const bool accepted = m_executor.interrupt_active_routine_if_possible();
    log_goal_event(
      accepted ? CommanderEventType::INTERRUPTION_ACCEPTED :
      CommanderEventType::INTERRUPTION_REJECTED,
      *m_active_goal,
      accepted ? "" : "Replacement goal remains pending");
    return accepted;
  }

  void activate_goal_and_start_routine(FlightGoalRequest request)
  {
    m_active_goal = std::move(request);
    log_goal_event(CommanderEventType::GOAL_STARTED, *m_active_goal);
    m_executor.start_routine_for_goal(
      m_active_goal->id(), m_active_goal->goal(), m_active_goal->retry_policy());
  }

  void set_next_goal(FlightGoalRequest request)
  {
    if (m_next_goal) {
      finish_goal(
        *m_next_goal,
        {GoalOutcome::INTERRUPTED, "Replaced by a newer flight goal"});
    }
    m_next_goal = std::move(request);
  }

  void start_next_goal_if_present()
  {
    if (!m_next_goal) return;
    auto next = std::move(*m_next_goal);
    m_next_goal.reset();
    activate_goal_and_start_routine(std::move(next));
  }

  void finish_goal_as_failed(FlightGoalRequest & request, std::string message)
  {
    finish_goal(request, {GoalOutcome::FAILURE, std::move(message)});
  }

  void finish_goal(FlightGoalRequest & request, GoalResult result)
  {
    const auto event = result.outcome == GoalOutcome::SUCCESS ?
      CommanderEventType::GOAL_SUCCEEDED :
      result.outcome == GoalOutcome::INTERRUPTED ?
      CommanderEventType::GOAL_INTERRUPTED : CommanderEventType::GOAL_FAILED;
    log_goal_event(event, request, result.message);
    request.complete_with_result(std::move(result));
  }

  void log_goal_event(
    CommanderEventType type,
    const FlightGoalRequest & request,
    std::string detail = {})
  {
    m_event_log->emit({type, request.id(), kind(request.goal()), std::move(detail)});
  }

  IFlightGoalExecutor & m_executor;
  std::shared_ptr<ICommanderEventSink> m_event_log;
  std::optional<FlightGoalRequest> m_active_goal;
  std::optional<FlightGoalRequest> m_next_goal;
  std::vector<FlightGoalRequest> m_deferred_cancelled_goals;
  bool m_interruption_in_progress{false};
  std::uint64_t m_next_goal_id{1};
  mutable std::recursive_mutex m_mutex;
  std::condition_variable_any m_no_active_goal;
};

}  // namespace padflies_cpp::commander
