#pragma once

#include "padflies_cpp/commander/commander_watchdog.hpp"

namespace padflies_cpp {
// Recovery selection belongs to the commander, not fault detection or the actor.
inline DeactivationMode deactivation_mode_for(HealthFault fault, CommanderState state)
{
    if (fault == HealthFault::CONTROL_COMMANDS_STOPPED)
        return DeactivationMode::RETURN;
    if (fault == HealthFault::HARDWARE_STATE_UPDATES_STOPPED)
        return DeactivationMode::RETURN;
    return DeactivationMode::STOP;
}
}  // namespace padflies_cpp
