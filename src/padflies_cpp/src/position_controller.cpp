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
    Eigen::Vector3d & target_position)
{
    m_clip_velocity(current_position, target_position);
    m_fade_target(target_position);
    m_clip_box(target_position);
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
    double xy_velocity = (target - position).head<2>().norm();
    double z_velocity = std::abs((target - position).z());

    auto direction = (target - position).normalized();
    // Linearly interpolate max_velocity_tick between m_max_xy_velocity_tick and m_max_z_velocity_tick
    double xy_ratio = xy_velocity / (xy_velocity + z_velocity);
    double max_velocity_tick = xy_ratio * m_max_xy_velocity_tick + (1.0 - xy_ratio) * m_max_z_velocity_tick;
    target = position + direction * max_velocity_tick;
}

void PositionController::m_fade_target(
    Eigen::Vector3d & target)
{
    m_target_history.insert(m_target_history.begin(), target); 
    if (m_target_history.size() > m_target_history_size) {
        m_target_history.resize(m_target_history_size);
    }

    size_t ln = m_target_history.size();
    std::vector<double> weights_xy(ln);
    std::vector<double> weights_z(ln);
    
    for (size_t i = 0; i < ln; ++i) {
        weights_xy[i] = std::pow((ln - i) / static_cast<double>(ln), 5.0); // smooth fade for XY
        weights_z[i] = std::pow((ln - i) / static_cast<double>(ln), 2.0);  // smoother fade for Z
    }

    double weight_sum_xy = std::accumulate(weights_xy.begin(), weights_xy.end(), 0.0);
    double weight_sum_z = std::accumulate(weights_z.begin(), weights_z.end(), 0.0);

    Eigen::Vector2d smoothed_xy = Eigen::Vector2d::Zero();
    double smoothed_z = 0.0;
    for (size_t i = 0; i < ln; ++i) {
        smoothed_xy += m_target_history[i].head<2>() * weights_xy[i];
        smoothed_z += m_target_history[i].z() * weights_z[i];
    }
    smoothed_xy /= weight_sum_xy;
    smoothed_z /= weight_sum_z;
    
    target.x() = smoothed_xy.x();
    target.y() = smoothed_xy.y();
    target.z() = smoothed_z;
}