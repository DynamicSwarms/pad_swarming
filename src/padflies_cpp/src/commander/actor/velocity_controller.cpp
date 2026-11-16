#include "padflies_cpp/commander/actor/velocity_controller.hpp"

VelocityController::VelocityController(
    double max_velocity,
    const std::vector<double> & clipping_box)
: m_max_velocity(max_velocity)
, m_clipping_box(clipping_box)
{
}

void VelocityController::safe_command_velocity(
    const Eigen::Vector3d & current_position,
    Eigen::Vector3d & target_velocity)
{
    m_clip_velocity(target_velocity);
    // Stop outward motion at the flight boundary while allowing inward motion.
    m_clip_box(current_position, target_velocity);
}

void VelocityController::m_clip_velocity(Eigen::Vector3d & target_velocity)
{
    const double velocity = target_velocity.norm();
    if (velocity > m_max_velocity)
    {
        target_velocity *= m_max_velocity / velocity;
    }
}

void VelocityController::m_clip_box(
    const Eigen::Vector3d & current_position,
    Eigen::Vector3d & target_velocity)
{
    for (Eigen::Index axis = 0; axis < 3; ++axis)
    {
        const bool outside_upper_bound =
            current_position(axis) >= m_clipping_box[axis];
        const bool outside_lower_bound =
            current_position(axis) <= m_clipping_box[axis + 3];

        if ((outside_upper_bound && target_velocity(axis) > 0.0) ||
            (outside_lower_bound && target_velocity(axis) < 0.0))
        {
            target_velocity(axis) = 0.0;
        }
    }
}
