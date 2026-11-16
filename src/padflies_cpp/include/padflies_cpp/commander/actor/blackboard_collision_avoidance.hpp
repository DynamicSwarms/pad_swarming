#pragma once

#include "rclcpp/rclcpp.hpp"
#include "collision_avoidance_interfaces/i_collision_avoidance.hpp"
#include "collision_avoidance_interfaces/msg/collision_states.hpp"
#include <Eigen/Dense>
#include <mutex>
#include <optional>

// World-frame state transport and injected local calculation, without service waits.
class BlackboardCollisionAvoidance
{
public:
    BlackboardCollisionAvoidance(
        uint8_t id,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics,
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock,
        std::shared_ptr<collision_avoidance_interfaces::ICollisionAvoidance> calculator);

    void get_collision_avoidance_velocity(
        const Eigen::Vector3d & position, Eigen::Vector3d & velocity,
        bool & collision, double radius = 0.15, double max_speed = 0.8,
        bool apply = true);
    void get_collision_avoidance_target(
        const Eigen::Vector3d & position, Eigen::Vector3d & target,
        bool & collision, bool apply = true);

private:
    std::shared_ptr<collision_avoidance_interfaces::ICollisionAvoidance> m_calculator;
    uint8_t m_id;
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> m_parameters;
    rclcpp::Clock::SharedPtr m_clock;
    // Snapshot and last command can be accessed by different executor threads.
    std::mutex m_mutex;
    std::mutex m_command_mutex;
    std::vector<collision_avoidance_interfaces::msg::CollisionState> m_states;
    Eigen::Vector2d m_last_velocity{Eigen::Vector2d::Zero()};
    std::optional<rclcpp::Time> m_last_stamp;
    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    rclcpp::Publisher<collision_avoidance_interfaces::msg::CollisionState>::SharedPtr m_publisher;
    rclcpp::Subscription<collision_avoidance_interfaces::msg::CollisionStates>::SharedPtr m_subscription;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr m_parameter_callback;
};
