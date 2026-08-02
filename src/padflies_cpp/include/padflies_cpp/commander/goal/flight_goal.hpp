#pragma once

#include <string>
#include <type_traits>
#include <variant>

#include "geometry_msgs/msg/pose_stamped.hpp"

namespace padflies_cpp::commander
{

struct Deploy
{};

struct DeployTo
{
  geometry_msgs::msg::PoseStamped target;
};

struct Return
{};

struct ReturnTo
{
  std::string site;
};

using FlightGoal = std::variant<Deploy, DeployTo, Return, ReturnTo>;

enum class FlightGoalKind
{
  DEPLOY,
  DEPLOY_TO,
  RETURN,
  RETURN_TO
};

enum class GoalOutcome
{
  SUCCESS,
  FAILURE,
  INTERRUPTED
};

struct GoalResult
{
  GoalOutcome outcome{GoalOutcome::FAILURE};
  std::string message;
};

enum class GoalPolicy
{
  REPLACEABLE,
  LOCKED
};

enum class GoalRetryPolicy
{
  NEVER,
  INFINITE
};

inline FlightGoalKind kind(const FlightGoal & goal)
{
  return std::visit(
    [](const auto & value) {
      using Goal = std::decay_t<decltype(value)>;
      if constexpr (std::is_same_v<Goal, Deploy>) return FlightGoalKind::DEPLOY;
      if constexpr (std::is_same_v<Goal, DeployTo>) return FlightGoalKind::DEPLOY_TO;
      if constexpr (std::is_same_v<Goal, Return>) return FlightGoalKind::RETURN;
      return FlightGoalKind::RETURN_TO;
    }, goal);
}

}  // namespace padflies_cpp::commander
