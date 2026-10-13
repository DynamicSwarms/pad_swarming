#pragma once

#include "class_loader/class_loader_core.hpp"
#include "rclcpp/rclcpp.hpp"
#include "collision_avoidance_interfaces/msg/collision_state.hpp"
#include <vector>

namespace collision_avoidance_interfaces
{
// Calculation only; transport, timestamps and freshness belong to the actor module.
class ICollisionAvoidance
{
public:
    virtual ~ICollisionAvoidance() = default;
    virtual geometry_msgs::msg::Vector3 calculate_velocity(
        const msg::CollisionState & self,
        const std::vector<msg::CollisionState> & neighbors,
        bool & constrained) = 0;
};
}

// Match the constructor-injection pattern used by pad resource managers.
namespace class_loader
{
template<>
struct InterfaceTraits<collision_avoidance_interfaces::ICollisionAvoidance>
{
    using constructor_parameters = ConstructorParameters<
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface>>;
};
}
