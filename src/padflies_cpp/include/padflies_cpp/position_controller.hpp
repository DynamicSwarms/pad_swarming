
#include <vector>

#include <Eigen/Dense>


/**
 * The position_controller provides functionality to convert target position setpoints to safe ones, which can be directly sent to the Crazyflie.
 */
class PositionController
{
public:
    PositionController(
        double dt,
        double max_xy_velocity,
        double max_z_velocity,
        const std::vector<double> & clipping_box
    );

    void safe_command_position(
        const Eigen::Vector3d & current_position,
        Eigen::Vector3d & target_position
    );

private: 
    void m_clip_box(
        Eigen::Vector3d & target
    );

    void m_clip_velocity(
        const Eigen::Vector3d & position,
        Eigen::Vector3d & target
    );

    void m_fade_target(
        Eigen::Vector3d & target
    );

private:
    double m_dt; // Time step in seconds
    double m_max_xy_velocity_tick; // Maximum XY velocity per tick (tick is m_dt seconds)
    double m_max_z_velocity_tick; // Maximum Z velocity per tick (tick is m_dt seconds)
    std::vector<double> m_clipping_box; // Clipping box for the position

    size_t m_target_history_size = 20; // Size of the target history for fading
    std::vector<Eigen::Vector3d> m_target_history; // History of target positions for fading
};



