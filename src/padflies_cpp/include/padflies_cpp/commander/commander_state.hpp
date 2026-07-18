#pragma once


enum class CommanderState {
    UNCONFIGURED,
    CONFIGURED,
    CHARGING, 
    CHARGED,
    TAKEOFF,
    FLYING,
    LANDING,
};