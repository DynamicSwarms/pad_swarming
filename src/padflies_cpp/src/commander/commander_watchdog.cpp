#include "padflies_cpp/commander/commander_watchdog.hpp"

#include <cmath>
#include <stdexcept>

namespace {
padflies_cpp::CommanderWatchdog::Config watchdog_config(
    const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters)
{
    const auto timeout = [&](const std::string & name, double seconds) {
        auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
        descriptor.read_only = true;
        const double value = parameters->declare_parameter(
            name, rclcpp::ParameterValue(seconds), descriptor).get<double>();
        if (!std::isfinite(value) || value <= 0.0)
            throw std::invalid_argument(name + " must be positive and finite");
        return rclcpp::Duration::from_seconds(value);
    };
    return {
        timeout("watchdog.hardware_state_timeout", 2.0),
        timeout("watchdog.control_command_timeout", 5.0),
        timeout("watchdog.takeoff_confirmation_timeout", 5.0)};
}
}  // namespace

namespace padflies_cpp {
CommanderWatchdog::CommanderWatchdog(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters)
: CommanderWatchdog(watchdog_config(parameters))
{}

void CommanderWatchdog::set_state(
    CommanderState state, std::uint64_t sequence, rclcpp::Time now)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    observe_time(now);
    if (state == m_state) return;

    const bool was_inactive = !active();
    m_state = state;
    m_state_since = now;
    m_state_sequence = sequence;
    m_flight_confirmed = false;
    if (was_inactive || state == CommanderState::FLYING) m_last_control = now;
}

void CommanderWatchdog::control_received(rclcpp::Time now)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    observe_time(now);
    m_last_control = now;
}

void CommanderWatchdog::update_flight_confirmation(
    const HardwareStateSnapshot & hardware, rclcpp::Time now)
{
    const bool expects_flight = m_state == CommanderState::FLYING;
    const bool new_hardware_state = hardware.sequence > m_state_sequence;
    const bool current_sample = hardware.received_at <= now &&
        now - hardware.received_at < m_config.telemetry_timeout;

    if (expects_flight && new_hardware_state && current_sample &&
        hardware.can_fly && !hardware.tumbled && hardware.flying)
        m_flight_confirmed = true;
}

HealthStatus CommanderWatchdog::check(
    const HardwareStateSnapshot & hardware, rclcpp::Time now)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    observe_time(now);
    /*
     * Monitor activated commanders only; inactive hardware need not report readiness.
     */
    if (!active()) return {};
    discard_hardware_state_after_clock_reset(hardware);
    update_flight_confirmation(hardware, now);

    const bool expects_flight = m_state == CommanderState::FLYING;
    const bool routine_controls_flight =
        m_state == CommanderState::TAKEOFF || m_state == CommanderState::LANDING;
    const bool new_hardware_state = hardware.sequence > m_state_sequence;

    const auto last_hardware_update = new_hardware_state ? hardware.received_at : m_state_since;
    const bool hardware_state_expired = now - last_hardware_update >= m_config.telemetry_timeout;
    const bool confirmation_expired = now - m_state_since >= m_config.confirmation_timeout;
    const bool control_commands_expired = now - m_last_control >= m_config.control_timeout;

    /*
     * Crazyflie does not send valid /state updates.
     * This might be because of a critical radio link failure, or the logging system has messed up.  
     */
    if (hardware_state_expired)
        return {HealthFault::HARDWARE_STATE_UPDATES_STOPPED};

    /*
     * Tumbling is unrecoverable, regardless of anything else.
     */
    if (hardware.tumbled)
        return {HealthFault::HARDWARE_IS_TUMBLED};

    /*
     * Losing flight permission in crazyflie supervisor is unrecoverable.
     * Ignore it while grounded, where moving the Crazyflie may legitimately make it unable to fly during movement.
     */
    if (expects_flight && !hardware.can_fly)
        return {HealthFault::EXPECTED_FLYING_BUT_CANNOT_FLY};

    /*
     * Flight was confirmed but hardware no longer reports flying. 
     * Probably a bug in the state machine. 
     * A crash should have triggered tumbled or cant fly guards.
     */
    if (expects_flight && m_flight_confirmed && !hardware.flying)
        return {HealthFault::EXPECTED_FLYING_BUT_NOT_FLYING};

    /*
     * Routine success may precede hardware confirmation, so allow a grace period.
     * If flight is never confirmed, treat it like a crash.
     */
    if (expects_flight && !m_flight_confirmed && confirmation_expired)
        return {HealthFault::TAKEOFF_NOT_CONFIRMED};

    /*
     * An activated padflie should receive control commands regularly.
     * Otherwise deactivate and be available for a new connection.
     * Hover targets should be sent periodically, even if unchanged.
     * Suspend this check during takeoff/landing; entering FLYING resets the window.
     */
    if (!routine_controls_flight && control_commands_expired)
        return {HealthFault::CONTROL_COMMANDS_STOPPED};

    return {};
}

bool CommanderWatchdog::active() const
{
    return m_state != CommanderState::UNCONFIGURED && m_state != CommanderState::CONFIGURED;
}

void CommanderWatchdog::observe_time(rclcpp::Time now)
{
    // A simulation reset starts new grace periods and invalidates old samples.
    if (m_last_now && (now.get_clock_type() != m_last_now->get_clock_type() || now < *m_last_now)) {
        m_state_since = now;
        m_last_control = now;
        m_flight_confirmed = false;
        m_rebaseline = true;
    }
    m_last_now = now;
}

void CommanderWatchdog::discard_hardware_state_after_clock_reset(
    const HardwareStateSnapshot & hardware)
{
    if (!m_rebaseline) return;
    m_state_sequence = hardware.sequence;
    m_rebaseline = false;
}

}  // namespace padflies_cpp
