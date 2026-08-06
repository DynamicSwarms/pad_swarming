#pragma once

#include <vector>

#include <Eigen/Dense>

/**
 * Converts velocity setpoints to values that can be safely sent to the Crazyflie.
 */
class VelocityController
{
public:
    VelocityController(
        double max_velocity,
        const std::vector<double> & clipping_box
    );

    void safe_command_velocity(
        const Eigen::Vector3d & current_position,
        Eigen::Vector3d & target_velocity
    );

private:
    void m_clip_box(
        const Eigen::Vector3d & current_position,
        Eigen::Vector3d & target_velocity
    );

    void m_clip_velocity(Eigen::Vector3d & target_velocity);

private:
    double m_max_velocity;
    std::vector<double> m_clipping_box;
};
