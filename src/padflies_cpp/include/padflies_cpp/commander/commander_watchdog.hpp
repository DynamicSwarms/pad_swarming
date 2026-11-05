#pragma once

#include "rclcpp/duration.hpp"
#include "padflies_cpp/commander/deactivation_mode.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include <optional>
#include <mutex>
#include "padflies_cpp/commander/commander_state.hpp"
#include "padflies_cpp/commander/actor/hardware_state_snapshot.hpp"

namespace padflies_cpp {
enum class HealthFault {
    NONE,
    EXPECTED_FLYING_BUT_NOT_FLYING,
    EXPECTED_FLYING_BUT_CANNOT_FLY,
    HARDWARE_IS_TUMBLED,
    TAKEOFF_NOT_CONFIRMED,
    HARDWARE_STATE_UPDATES_STOPPED,
    CONTROL_COMMANDS_STOPPED
};

inline const char * fault_name(HealthFault fault) {
    switch (fault) {
#define FAULT_NAME(value) case HealthFault::value: return #value
        FAULT_NAME(NONE);
        FAULT_NAME(EXPECTED_FLYING_BUT_NOT_FLYING);
        FAULT_NAME(EXPECTED_FLYING_BUT_CANNOT_FLY);
        FAULT_NAME(HARDWARE_IS_TUMBLED);
        FAULT_NAME(TAKEOFF_NOT_CONFIRMED);
        FAULT_NAME(HARDWARE_STATE_UPDATES_STOPPED);
        FAULT_NAME(CONTROL_COMMANDS_STOPPED);
#undef FAULT_NAME
    }
    return "UNKNOWN";
}

struct HealthStatus {
    HealthFault fault = HealthFault::NONE;
    DeactivationMode deactivation_mode = DeactivationMode::STOP;
    bool healthy() const { return fault == HealthFault::NONE; }
};

class CommanderWatchdog {
public:
    struct Config {
        rclcpp::Duration telemetry_timeout = rclcpp::Duration::from_seconds(2);
        rclcpp::Duration control_timeout = rclcpp::Duration::from_seconds(5);
        rclcpp::Duration confirmation_timeout = rclcpp::Duration::from_seconds(5);
    };

    explicit CommanderWatchdog(
        rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters);

    explicit CommanderWatchdog(Config config) : m_config(config) {}

    void set_state(CommanderState state, std::uint64_t sequence, rclcpp::Time now);
    void control_received(rclcpp::Time now);
    HealthStatus check(const HardwareStateSnapshot & hardware, rclcpp::Time now);

private:
    void update_flight_confirmation(const HardwareStateSnapshot & hardware, rclcpp::Time now);
    bool active() const;
    void observe_time(rclcpp::Time now);
    void discard_hardware_state_after_clock_reset(const HardwareStateSnapshot & hardware);

    std::optional<rclcpp::Time> m_last_now;
    bool m_rebaseline = false;
    Config m_config;
    std::mutex m_mutex;
    CommanderState m_state = CommanderState::UNCONFIGURED;
    rclcpp::Time m_state_since{0, 0, RCL_ROS_TIME};
    rclcpp::Time m_last_control{0, 0, RCL_ROS_TIME};
    std::uint64_t m_state_sequence = 0;
    bool m_flight_confirmed = false;
};
}  // namespace padflies_cpp
