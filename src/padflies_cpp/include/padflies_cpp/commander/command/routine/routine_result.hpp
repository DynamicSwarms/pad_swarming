#pragma once

#include <functional>
#include <string>

namespace BT
{
class Tree;
}

enum class RoutineOutcome
{
  SUCCESS,
  FAILURE,
  RETRY,
  INTERRUPTED
};

enum class RoutineFailureReason
{
  NONE,
  SITE,
  HARDWARE,
  INTERNAL
};

struct RoutineResult
{
  RoutineOutcome outcome = RoutineOutcome::FAILURE;
  RoutineFailureReason reason = RoutineFailureReason::NONE;
  std::string message;
};

using RoutineResultClassifier = std::function<RoutineResult(const BT::Tree &)>;
