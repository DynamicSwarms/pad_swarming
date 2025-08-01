#include "padflies_cpp/position_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include <numeric>
PositionController::PositionController(
    double dt,
    double max_xy_velocity,
    double max_z_velocity,
    const std::vector<double> & clipping_box)
: m_dt(dt)
, m_max_xy_velocity_tick(m_dt * max_xy_velocity)
, m_max_z_velocity_tick(m_dt * max_z_velocity)
, m_clipping_box(clipping_box)
{
}   

void PositionController::safe_command_position(
    const Eigen::Vector3d & current_position,
    const Eigen::Vector3d & target_position,
    Eigen::Vector3d & safe_position)
{
    // Start with the target position
    safe_position = target_position;
    m_fade_target(safe_position);
    m_clip_velocity(current_position, safe_position);
    m_clip_box(safe_position);
}

void PositionController::m_clip_box(
    Eigen::Vector3d & target)
{
   target = Eigen::Vector3d(
        std::min(m_clipping_box[0], std::max(m_clipping_box[3], target.x())),   // x
        std::min(m_clipping_box[1], std::max(m_clipping_box[4], target.y())),   // y
        std::min(m_clipping_box[2], std::max(m_clipping_box[5], target.z())));  // z
}

void PositionController::m_clip_velocity(
    const Eigen::Vector3d & position,
    Eigen::Vector3d & target)
{
    // Clip the XY velocity
    double xy_velocity = (target - position).head<2>().norm();
 
    if (xy_velocity > m_max_xy_velocity_tick) {
        auto direction = (target - position).head<2>().normalized();
        target.head<2>() = position.head<2>() + direction * m_max_xy_velocity_tick;
    }

    // Clip the Z velocity
    if (std::abs((target-position).z()) > m_max_z_velocity_tick) {
        auto direction = (target - position).z() > 0 ? 1.0 : -1.0;
        target.z() = position.z() + direction * m_max_z_velocity_tick;
    }
}

void PositionController::m_fade_target(
    Eigen::Vector3d & target)
{
    m_target_history.insert(m_target_history.begin(), target); 
    if (m_target_history.size() > m_target_history_size) {
        m_target_history.resize(m_target_history_size);
    }

    size_t ln = m_target_history.size();
    std::vector<double> weights(ln);
    
    for (size_t i = 0; i < ln; ++i) {
        weights[i] = std::pow((ln - i) / static_cast<double>(ln), 2.6);
    }

    double weight_sum = std::accumulate(weights.begin(), weights.end(), 0.0);
    target = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < ln; ++i) {
        target += m_target_history[i] * weights[i];
    }
    target /= weight_sum;
}