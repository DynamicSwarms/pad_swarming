#pragma once

#include "padflies_cpp/commander/goal/flight_goal.hpp"

namespace padflies_cpp::commander
{

class IGoalCompletion
{
public:
  virtual ~IGoalCompletion() = default;
  virtual void complete(const GoalResult & result) = 0;
};

}  // namespace padflies_cpp::commander
