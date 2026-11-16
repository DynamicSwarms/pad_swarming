#include "padflies_cpp/commander/actor/blackboard_collision_avoidance.hpp"
#include <cmath>
#include <stdexcept>

namespace
{
Eigen::Vector2d xy(const geometry_msgs::msg::Vector3 & vector)
{
    return {vector.x, vector.y};
}
geometry_msgs::msg::Vector3 vector_msg(const Eigen::Vector3d & vector)
{
    geometry_msgs::msg::Vector3 result;
    result.x = vector.x(); result.y = vector.y(); result.z = vector.z();
    return result;
}
bool valid(const collision_avoidance_interfaces::msg::CollisionState & state)
{
    return xy(state.position).allFinite() && xy(state.velocity).allFinite() &&
        xy(state.preferred_velocity).allFinite() && xy(state.calculated_velocity).allFinite() &&
        std::isfinite(state.radius) && state.radius > 0.0 &&
        std::isfinite(state.max_speed) && state.max_speed > 0.0;
}
}

BlackboardCollisionAvoidance::BlackboardCollisionAvoidance(
    uint8_t id,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock,
    std::shared_ptr<collision_avoidance_interfaces::ICollisionAvoidance> calculator)
    : m_calculator(calculator), m_id(id), m_parameters(parameters), m_clock(clock->get_clock())
{
    if (!m_calculator) throw std::invalid_argument("Collision avoidance calculator is required");
    // Actors are recreated on activation; retain the node's existing parameters.
    const auto declare_positive = [&](const std::string & name, double value) {
        if (!parameters->has_parameter(name)) {
            parameters->declare_parameter(name, rclcpp::ParameterValue(value));
        }
        const auto current = parameters->get_parameter(name).as_double();
        if (!std::isfinite(current) || current <= 0.0) {
            throw std::invalid_argument(name + " must be positive and finite");
        }
    };
    declare_positive("collision_avoidance.max_state_age", 0.2);
    if (!parameters->has_parameter("collision_avoidance.apply")) {
        parameters->declare_parameter("collision_avoidance.apply", rclcpp::ParameterValue(true));
    }
    parameters->get_parameter("collision_avoidance.apply").as_bool();
    m_parameter_callback = parameters->add_on_set_parameters_callback(
        [](const std::vector<rclcpp::Parameter> & updates) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            for (const auto & parameter : updates) {
                const auto & name = parameter.get_name();
                if (name == "collision_avoidance.max_state_age") {
                    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE ||
                        !std::isfinite(parameter.as_double()) || parameter.as_double() <= 0.0) {
                        result.successful = false;
                        result.reason = name + " must be a positive finite double";
                        break;
                    }
                } else if (name == "collision_avoidance.apply" &&
                    parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
                    result.successful = false;
                    result.reason = name + " must be a boolean";
                    break;
                }
            }
            return result;
        });
    m_callback_group = base->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_publisher = rclcpp::create_publisher<collision_avoidance_interfaces::msg::CollisionState>(
        topics, "/collision_avoidance/state", rclcpp::QoS(256));
    rclcpp::SubscriptionOptions options;
    options.callback_group = m_callback_group;
    m_subscription = rclcpp::create_subscription<collision_avoidance_interfaces::msg::CollisionStates>(
        topics, "/collision_avoidance/states", rclcpp::QoS(1),
        [this](collision_avoidance_interfaces::msg::CollisionStates::ConstSharedPtr snapshot) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_states = snapshot->states;
        }, options);
}

void BlackboardCollisionAvoidance::get_collision_avoidance_velocity(
    const Eigen::Vector3d & position, Eigen::Vector3d & velocity,
    bool & collision, double radius, double max_speed, bool apply)
{
    std::lock_guard<std::mutex> command_lock(m_command_mutex);
    collision = false;
    if (!position.allFinite() || !velocity.allFinite() ||
        !std::isfinite(radius) || radius <= 0.0 || !std::isfinite(max_speed) || max_speed <= 0.0) {
        velocity.setZero();
        collision = true;
        return;
    }
    const auto now = m_clock->now();
    const double max_age = m_parameters->get_parameter("collision_avoidance.max_state_age").as_double();
    const Eigen::Vector3d preferred = velocity;
    Eigen::Vector2d current = preferred.head<2>();
    if (m_last_stamp) {
        const double age = (now - *m_last_stamp).seconds();
        if (age >= 0.0 && age <= max_age) current = m_last_velocity;
    }
    collision_avoidance_interfaces::msg::CollisionState self;
    self.id = m_id; self.stamp = now;
    self.position = vector_msg(position);
    self.preferred_velocity = vector_msg(preferred);
    self.velocity = vector_msg(Eigen::Vector3d(current.x(), current.y(), preferred.z()));
    self.calculated_velocity = self.velocity;
    self.radius = radius; self.max_speed = max_speed;
    std::vector<collision_avoidance_interfaces::msg::CollisionState> states;
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        states = m_states;
    }
    std::vector<collision_avoidance_interfaces::msg::CollisionState> neighbors;
    neighbors.reserve(states.size());
    for (const auto & state : states) {
        if (state.id == m_id || !valid(state)) continue;
        const auto stamp = rclcpp::Time(state.stamp, now.get_clock_type());
        const double age = (now - stamp).seconds();
        if (age < 0.0 || age > max_age) continue;
        neighbors.push_back(state);
    }
    bool constrained = false;
    const auto calculated = m_calculator->calculate_velocity(self, neighbors, constrained);
    if (!std::isfinite(calculated.x) || !std::isfinite(calculated.y)) {
        velocity.setZero();
        collision = true;
        return;
    }
    const bool enabled = apply &&
        m_parameters->get_parameter("collision_avoidance.apply").as_bool();
    if (enabled) velocity.head<2>() = Eigen::Vector2d(calculated.x, calculated.y);
    collision = enabled && constrained;
    m_last_velocity = velocity.head<2>();
    m_last_stamp = now;

    collision_avoidance_interfaces::msg::CollisionState state;
    state.id = m_id;
    state.stamp = now;
    state.position = vector_msg(position);
    state.preferred_velocity = vector_msg(preferred);
    state.velocity = vector_msg(velocity);
    state.calculated_velocity = calculated;
    state.radius = radius;
    state.max_speed = max_speed;
    m_publisher->publish(state);
}

void BlackboardCollisionAvoidance::get_collision_avoidance_target(
    const Eigen::Vector3d & position, Eigen::Vector3d & target, bool & collision, bool apply)
{
    // Preserve the previous target-to-velocity conversion, including unchanged Z.
    Eigen::Vector3d velocity = target - position;
    constexpr double max_speed = 0.8;
    if (velocity.head<2>().norm() > max_speed) {
        velocity.head<2>() = velocity.head<2>().normalized() * max_speed;
    }
    get_collision_avoidance_velocity(position, velocity, collision, 0.15, max_speed, apply);
    if (apply && m_parameters->get_parameter("collision_avoidance.apply").as_bool()) {
        target.x() = position.x() + velocity.x();
        target.y() = position.y() + velocity.y();
    }
}
