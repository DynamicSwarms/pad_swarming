#pragma once

#include <cstdint>

#include "padflies_cpp/commander/goal/flight_goal.hpp"

namespace padflies_cpp::commander
{

class IFlightGoalExecutor
{
public:
  virtual ~IFlightGoalExecutor() = default;

  virtual void start_routine_for_goal(
    std::uint64_t goal_id,
    const FlightGoal & goal,
    GoalRetryPolicy retry_policy) = 0;

  virtual bool interrupt_active_routine_if_possible() = 0;
  virtual void stop_active_routine_without_callback() = 0;
};

}  // namespace padflies_cpp::commander
