#pragma once
#include "collision_avoidance_interfaces/i_collision_avoidance.hpp"

namespace collision_avoidance
{
class Orca : public collision_avoidance_interfaces::ICollisionAvoidance
{
public:
    explicit Orca(std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters);
    geometry_msgs::msg::Vector3 calculate_velocity(
        const collision_avoidance_interfaces::msg::CollisionState & self,
        const std::vector<collision_avoidance_interfaces::msg::CollisionState> & neighbors,
        bool & constrained) override;
private:
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> m_parameters;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameter_callback;
};
}
