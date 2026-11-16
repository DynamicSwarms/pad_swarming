#pragma once

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>

#include "padflies_cpp/commander/goal/completion/goal_completion.hpp"

namespace padflies_cpp::commander
{

class FlightGoalRequest
{
public:
  FlightGoalRequest(
    std::uint64_t id,
    FlightGoal goal,
    std::shared_ptr<IGoalCompletion> completion,
    GoalRetryPolicy retry_policy,
    GoalPolicy policy)
  : m_id(id),
    m_goal(std::move(goal)),
    m_completion(std::move(completion)),
    m_retry_policy(retry_policy),
    m_policy(policy)
  {
    if (!m_completion) {
      throw std::invalid_argument("A flight goal requires a completion handler");
    }
  }

  std::uint64_t id() const noexcept {return m_id;}
  const FlightGoal & goal() const noexcept {return m_goal;}
  GoalRetryPolicy retry_policy() const noexcept {return m_retry_policy;}
  bool can_be_replaced() const noexcept {return m_policy == GoalPolicy::REPLACEABLE;}

  void complete_with_result(GoalResult result)
  {
    if (m_finished) {
      throw std::logic_error("A flight goal was finished twice");
    }
    m_finished = true;
    m_completion->complete(result);
  }

private:
  std::uint64_t m_id;
  FlightGoal m_goal;
  std::shared_ptr<IGoalCompletion> m_completion;
  GoalRetryPolicy m_retry_policy;
  GoalPolicy m_policy;
  bool m_finished{false};
};

}  // namespace padflies_cpp::commander
