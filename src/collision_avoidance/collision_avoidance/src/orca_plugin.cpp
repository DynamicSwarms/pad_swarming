#include "collision_avoidance/orca_plugin.hpp"
#include "orca.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <cmath>
#include <stdexcept>

namespace collision_avoidance
{
Orca::Orca(std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters)
    : m_parameters(parameters)
{
    const auto declare_positive = [&](const std::string & name, double value) {
        if (!parameters->has_parameter(name)) {
            parameters->declare_parameter(name, rclcpp::ParameterValue(value));
        }
        const auto current = parameters->get_parameter(name).as_double();
        if (!std::isfinite(current) || current <= 0.0) {
            throw std::invalid_argument(name + " must be positive and finite");
        }
    };
    declare_positive("orca.time_step", 0.1);
    declare_positive("orca.time_horizon", 1.0);
    declare_positive("orca.neighbor_distance", 1.0);
    m_parameter_callback = parameters->add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> & updates) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto & parameter : updates) {
                const auto & name = parameter.get_name();
                if (name == "orca.time_step" || name == "orca.time_horizon" || name == "orca.neighbor_distance") {
                    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
                        !std::isfinite(parameter.as_double()) || parameter.as_double() <= 0.0) {
                        result.successful = false;
                        result.reason = name + " must be a positive finite double";
                        break;
                    }
                }
            }
            return result;
        });
}

geometry_msgs::msg::Vector3 Orca::calculate_velocity(
    const collision_avoidance_interfaces::msg::CollisionState & self,
    const std::vector<collision_avoidance_interfaces::msg::CollisionState> & neighbors,
    bool & constrained)
{
    const auto convert = [](const collision_avoidance_interfaces::msg::CollisionState & state) {
        return ObjectInfo{{state.position.x, state.position.y},
            {state.preferred_velocity.x, state.preferred_velocity.y},
            {state.velocity.x, state.velocity.y},
            {state.calculated_velocity.x, state.calculated_velocity.y},
            state.radius, state.max_speed, rclcpp::Time(state.stamp, RCL_ROS_TIME)};
    };
    std::vector<ObjectInfo> objects;
    objects.reserve(neighbors.size());
    for (const auto & neighbor : neighbors) objects.push_back(convert(neighbor));
    const auto velocity = calculate_orca_velocity(convert(self), objects,
        m_parameters->get_parameter("orca.time_horizon").as_double(),
        m_parameters->get_parameter("orca.time_step").as_double(),
        m_parameters->get_parameter("orca.neighbor_distance").as_double(), constrained);
    auto result = self.preferred_velocity;
    result.x = velocity.x(); result.y = velocity.y();
    return result;
}
}

PLUGINLIB_EXPORT_CLASS(collision_avoidance::Orca, collision_avoidance_interfaces::ICollisionAvoidance)
