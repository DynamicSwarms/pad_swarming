#include "padflies_cpp/yaw_controller.hpp"


YawController::YawController(
    double dt,
    double max_rotational_velocity)
: m_dt(dt)
, m_max_rotational_velocity_tick(m_dt * max_rotational_velocity)
{
}

double YawController::safe_cmd_yaw(
    double current_yaw,
    double target_yaw)
{
    // Normalize the angles
    current_yaw = m_normalize_angle(current_yaw);
    double shortest_angle = m_shortest_angle(current_yaw, target_yaw);

    if (std::abs(shortest_angle) > m_max_rotational_velocity_tick) {
        shortest_angle = m_max_rotational_velocity_tick * (shortest_angle > 0 ? 1 : -1);
    }
    return m_normalize_angle(current_yaw + shortest_angle);
 }

double YawController::m_normalize_angle(
    double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

double YawController::m_shortest_angle(
    double source,
    double target)
{
    return std::atan2(
        std::sin(target - source),
        std::cos(target - source));
}
