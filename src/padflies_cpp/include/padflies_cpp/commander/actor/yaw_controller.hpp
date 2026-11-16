#include "rclcpp/rclcpp.hpp"


class YawController
{
    public:
    YawController(
        double dt,
        double max_rotational_velocity
    );

    double safe_cmd_yaw(
        double current_yaw,
        double target_yaw
    );

private:
    /**
    * @brief Normalize the angle to the range [-pi, pi].
    * 
    * @param angle The angle in radians.
    * @return The normalized angle in radians. 
    */
    double m_normalize_angle(
        double angle
    );

    /**
     * @brief Comptes the shortest angle between the source and target angle.
     * The sign corresponds to the direction.
     * 
     * @param source The source angle in radians (to start from).
     * @param target The target angle in radians (to go to).
     * @return The angle difference, sign is the direction to go in.
     */
    double m_shortest_angle(
        double source,
        double target
    );

private:
    double m_dt; // Time step in seconds
    double m_max_rotational_velocity_tick; // Maximum rotational velocity per tick (tick is m_dt seconds)
};