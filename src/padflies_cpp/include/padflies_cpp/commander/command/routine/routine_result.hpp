#pragma once

#include <functional>
#include <string>

enum class RoutineTermination
{
  SUCCESS,
  FAILURE,
  SKIPPED,
  HALTED
};

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

using RoutineResultClassifier =
  std::function<RoutineResult(RoutineTermination)>;
