#pragma once

#include <cstdint>
#include <string>

#include "padflies_cpp/commander/goal/flight_goal.hpp"

namespace padflies_cpp::commander
{

enum class CommanderEventType
{
  GOAL_REQUESTED,
  GOAL_STARTED,
  INTERRUPTION_REQUESTED,
  INTERRUPTION_ACCEPTED,
  INTERRUPTION_REJECTED,
  GOAL_SUCCEEDED,
  GOAL_FAILED,
  GOAL_INTERRUPTED
};

struct CommanderEvent
{
  CommanderEventType type;
  std::uint64_t goal_id;
  FlightGoalKind goal_kind;
  std::string detail;
};

class ICommanderEventSink
{
public:
  virtual ~ICommanderEventSink() = default;
  virtual void emit(const CommanderEvent & event) = 0;
};

class NullCommanderEventSink final : public ICommanderEventSink
{
public:
  void emit(const CommanderEvent &) override {}
};

}  // namespace padflies_cpp::commander
