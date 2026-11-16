#include "padflies_cpp/commander/commander_watchdog.hpp"
#include "padflies_cpp/commander/deactivation_policy.hpp"
#include "rclcpp/clock.hpp"
#include "rcl/time.h"
#include <cstdlib>
#include <iostream>

using namespace padflies_cpp;

void require(bool condition) {
    if (!condition) { std::cerr << "Watchdog expectation failed\n"; std::abort(); }
}

int main() {
    const rclcpp::Time start(0, 0, RCL_ROS_TIME);
    CommanderWatchdog watchdog(CommanderWatchdog::Config{});
    HardwareStateSnapshot hardware{true, false, false, 1, start};
    require(watchdog.check(hardware, start).healthy());
    watchdog.set_state(CommanderState::CHARGED, 1, start);
    // Tumbling is actionable even before a post-transition sample arrives.
    hardware.tumbled = true;
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(1)).fault ==
        HealthFault::HARDWARE_IS_TUMBLED);
    hardware.tumbled = false;
    CommanderWatchdog missing(CommanderWatchdog::Config{});
    missing.set_state(CommanderState::CHARGED, 1, start);
    require(missing.check(hardware, start + rclcpp::Duration::from_seconds(2)).fault ==
        HealthFault::HARDWARE_STATE_UPDATES_STOPPED);
    hardware = {false, false, false, 2, start + rclcpp::Duration::from_seconds(1)};
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(1)).healthy());
    hardware.tumbled = true;
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(1)).fault == HealthFault::HARDWARE_IS_TUMBLED);

    hardware.tumbled = false;
    watchdog.set_state(CommanderState::FLYING, 2, start + rclcpp::Duration::from_seconds(1));
    // Flight permission is actionable without waiting for another sample.
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(1)).fault ==
        HealthFault::EXPECTED_FLYING_BUT_CANNOT_FLY);
    hardware.can_fly = true;
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(1)).healthy());
    hardware = {true, false, false, 3, start + rclcpp::Duration::from_seconds(2)};
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(2)).healthy());
    watchdog.control_received(start + rclcpp::Duration::from_seconds(5));
    hardware.sequence = 4;
    hardware.received_at = start + rclcpp::Duration::from_seconds(6);
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(6)).fault == HealthFault::TAKEOFF_NOT_CONFIRMED);
    hardware.flying = true;
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(6)).healthy());
    hardware.flying = false;
    hardware.sequence++;
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(6)).fault ==
        HealthFault::EXPECTED_FLYING_BUT_NOT_FLYING);
    hardware.can_fly = false;
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(6)).fault ==
        HealthFault::EXPECTED_FLYING_BUT_CANNOT_FLY);

    watchdog.set_state(CommanderState::LANDING, hardware.sequence, start + rclcpp::Duration::from_seconds(6));
    hardware = {false, false, false, 6, start + rclcpp::Duration::from_seconds(20)};
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(20)).healthy());
    watchdog.set_state(CommanderState::CONFIGURED, 6, start + rclcpp::Duration::from_seconds(20));
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(100)).healthy());

    watchdog.set_state(CommanderState::CHARGED, 6, start + rclcpp::Duration::from_seconds(100));
    hardware.sequence++;
    hardware.received_at = start + rclcpp::Duration::from_seconds(105);
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(105)).fault == HealthFault::CONTROL_COMMANDS_STOPPED);
    watchdog.control_received(start + rclcpp::Duration::from_seconds(105));
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(105)).healthy());
    // A charge-state transition must not renew external control activity.
    watchdog.set_state(CommanderState::CHARGING, hardware.sequence, start + rclcpp::Duration::from_seconds(109));
    hardware.sequence++;
    hardware.received_at = start + rclcpp::Duration::from_seconds(110);
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(110)).fault == HealthFault::CONTROL_COMMANDS_STOPPED);
    // A paused ROS clock does not consume timeout budgets.
    watchdog.control_received(start + rclcpp::Duration::from_seconds(110));
    for (int i = 0; i < 100; ++i)
        require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(110)).healthy());
    // Clock resets still preserve immediate safety faults.
    hardware.tumbled = true;
    require(watchdog.check(hardware, start).fault == HealthFault::HARDWARE_IS_TUMBLED);
    hardware.tumbled = false;
    // Reset grace periods still demand new telemetry.
    require(watchdog.check(hardware, start).healthy());
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(1)).healthy());
    require(watchdog.check(hardware, start + rclcpp::Duration::from_seconds(2)).fault ==
        HealthFault::HARDWARE_STATE_UPDATES_STOPPED);
    hardware.sequence++;
    hardware.tumbled = true;
    hardware.received_at = start + rclcpp::Duration::from_seconds(2);
    require(watchdog.check(hardware, hardware.received_at).fault == HealthFault::HARDWARE_IS_TUMBLED);
    // Exercise a ROS clock with simulation-time override, not wall time.
    rclcpp::Clock clock(RCL_ROS_TIME);
    require(rcl_enable_ros_time_override(clock.get_clock_handle()) == RCL_RET_OK);
    require(rcl_set_ros_time_override(clock.get_clock_handle(), 10000000000LL) == RCL_RET_OK);
    CommanderWatchdog simulated(CommanderWatchdog::Config{});
    simulated.set_state(CommanderState::CHARGED, 0, clock.now());
    HardwareStateSnapshot simulated_hardware{true, false, false, 1, clock.now()};
    for (int i = 0; i < 100; ++i)
        require(simulated.check(simulated_hardware, clock.now()).healthy());
    require(rcl_set_ros_time_override(clock.get_clock_handle(), 12000000000LL) == RCL_RET_OK);
    require(simulated.check(simulated_hardware, clock.now()).fault ==
        HealthFault::HARDWARE_STATE_UPDATES_STOPPED);
    require(rcl_set_ros_time_override(clock.get_clock_handle(), 0) == RCL_RET_OK);
    require(simulated.check(simulated_hardware, clock.now()).healthy());
    // Crashes and failed takeoff must never issue another flight maneuver.
    for (const auto state : {CommanderState::CHARGED, CommanderState::TAKEOFF,
                             CommanderState::FLYING, CommanderState::LANDING}) {
        for (const auto fault : {HealthFault::EXPECTED_FLYING_BUT_NOT_FLYING,
                                 HealthFault::EXPECTED_FLYING_BUT_CANNOT_FLY,
                                 HealthFault::HARDWARE_IS_TUMBLED,
                                 HealthFault::TAKEOFF_NOT_CONFIRMED})
            require(deactivation_mode_for(fault, state) == DeactivationMode::STOP);
        require(deactivation_mode_for(HealthFault::CONTROL_COMMANDS_STOPPED, state) ==
            DeactivationMode::RETURN);
    }
    for (const auto state : {CommanderState::TAKEOFF, CommanderState::FLYING,
                             CommanderState::LANDING})
        require(deactivation_mode_for(HealthFault::HARDWARE_STATE_UPDATES_STOPPED, state) ==
            DeactivationMode::FAIL_SAFE);
    for (const auto state : {CommanderState::CHARGING, CommanderState::CHARGED})
        require(deactivation_mode_for(HealthFault::HARDWARE_STATE_UPDATES_STOPPED, state) ==
            DeactivationMode::STOP);
    // Waiting for a post-transition sample must not disable other deadlines.
    CommanderWatchdog::Config waiting_config;
    waiting_config.telemetry_timeout = rclcpp::Duration::from_seconds(10);
    waiting_config.control_timeout = rclcpp::Duration::from_seconds(1);
    waiting_config.confirmation_timeout = rclcpp::Duration::from_seconds(1);
    CommanderWatchdog awaiting_takeoff(waiting_config);
    HardwareStateSnapshot cached{true, false, false, 1, start};
    awaiting_takeoff.set_state(CommanderState::FLYING, cached.sequence, start);
    require(awaiting_takeoff.check(cached, start + rclcpp::Duration::from_seconds(1)).fault ==
        HealthFault::TAKEOFF_NOT_CONFIRMED);
    CommanderWatchdog awaiting_control(waiting_config);
    awaiting_control.set_state(CommanderState::CHARGED, cached.sequence, start);
    require(awaiting_control.check(cached, start + rclcpp::Duration::from_seconds(1)).fault ==
        HealthFault::CONTROL_COMMANDS_STOPPED);
    // An expired flying sample must not establish flight confirmation.
    CommanderWatchdog::Config expired_config;
    expired_config.control_timeout = rclcpp::Duration::from_seconds(20);
    CommanderWatchdog expired_sample(expired_config);
    expired_sample.set_state(CommanderState::FLYING, 1, start);
    HardwareStateSnapshot old_flying{true, true, false, 2, start};
    require(expired_sample.check(old_flying, start + rclcpp::Duration::from_seconds(3)).fault ==
        HealthFault::HARDWARE_STATE_UPDATES_STOPPED);
    HardwareStateSnapshot new_grounded{true, false, false, 3,
        start + rclcpp::Duration::from_seconds(5)};
    require(expired_sample.check(new_grounded, new_grounded.received_at).fault ==
        HealthFault::TAKEOFF_NOT_CONFIRMED);
    // Sampled flight confirmation is remembered by subsequent checks.
    CommanderWatchdog between_checks(CommanderWatchdog::Config{});
    between_checks.set_state(CommanderState::FLYING, 1, start);
    HardwareStateSnapshot airborne{true, true, false, 2,
        start + rclcpp::Duration::from_seconds(1)};
    require(between_checks.check(airborne, airborne.received_at).healthy());
    HardwareStateSnapshot stopped{true, false, false, 3,
        start + rclcpp::Duration::from_seconds(1.1)};
    require(between_checks.check(stopped, stopped.received_at).fault ==
        HealthFault::EXPECTED_FLYING_BUT_NOT_FLYING);
    // Rechecking a snapshot cannot renew telemetry or control activity.
    require(between_checks.check(stopped, start + rclcpp::Duration::from_seconds(4)).fault ==
        HealthFault::HARDWARE_STATE_UPDATES_STOPPED);
    // Safety flags are actionable while current; telemetry loss wins once expired.
    for (auto sample : {HardwareStateSnapshot{true, true, true, 2, start},
                        HardwareStateSnapshot{false, true, false, 2, start}}) {
        CommanderWatchdog safety(CommanderWatchdog::Config{});
        safety.set_state(CommanderState::FLYING, 1, start);
        const auto safety_fault = sample.tumbled ? HealthFault::HARDWARE_IS_TUMBLED :
            HealthFault::EXPECTED_FLYING_BUT_CANNOT_FLY;
        require(safety.check(sample, start).fault == safety_fault);
        const auto expired_at = start + rclcpp::Duration::from_seconds(2);
        require(safety.check(sample, expired_at).fault == HealthFault::HARDWARE_STATE_UPDATES_STOPPED);
        sample.sequence++;
        sample.received_at = expired_at;
        require(safety.check(sample, expired_at).fault == safety_fault);
    }
    // No target stream is required while a departure/landing routine owns control.
    CommanderWatchdog routine_control(CommanderWatchdog::Config{});
    routine_control.set_state(CommanderState::CHARGED, 1, start);
    routine_control.control_received(start);
    routine_control.set_state(CommanderState::TAKEOFF, 1, start);
    HardwareStateSnapshot routine_state{true, false, false, 2,
        start + rclcpp::Duration::from_seconds(10)};
    require(routine_control.check(routine_state, routine_state.received_at).healthy());
    routine_control.set_state(CommanderState::FLYING, 2, routine_state.received_at);
    routine_state.sequence++;
    routine_state.flying = true;
    require(routine_control.check(routine_state, routine_state.received_at).healthy());
    routine_state.received_at = start + rclcpp::Duration::from_seconds(14.9);
    require(routine_control.check(routine_state, routine_state.received_at).healthy());
    routine_state.received_at = start + rclcpp::Duration::from_seconds(15);
    require(routine_control.check(routine_state, routine_state.received_at).fault ==
        HealthFault::CONTROL_COMMANDS_STOPPED);
    routine_control.set_state(CommanderState::LANDING, routine_state.sequence,
        routine_state.received_at);
    routine_state.sequence++;
    routine_state.received_at = start + rclcpp::Duration::from_seconds(25);
    routine_state.flying = false;
    require(routine_control.check(routine_state, routine_state.received_at).healthy());
}
