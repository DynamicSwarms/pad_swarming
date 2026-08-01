#pragma once

#include "rclcpp/rclcpp.hpp"

#include "collision_avoidance_interfaces/srv/collision_avoidance.hpp"
#include "collision_avoidance_interfaces/srv/velocity_reciprocals_collision_avoidance.hpp"

#include <Eigen/Dense>

class CollisionAvoidanceClient
{
public:
    CollisionAvoidanceClient(
        uint8_t cf_id,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        rclcpp::Logger logger);

    ~CollisionAvoidanceClient();

    void get_collision_avoidance_target(
        const Eigen::Vector3d & position,
        Eigen::Vector3d & target,
        bool & collision);

    void get_collision_avoidance_velocity(
        const Eigen::Vector3d & position,
        Eigen::Vector3d & velocity,
        bool & collision,
        double radius = 0.15,
        double max_speed = 5.0);

private: 
    uint8_t m_cf_id;
    std::shared_ptr<rclcpp::Client<collision_avoidance_interfaces::srv::CollisionAvoidance>> m_client;
    std::shared_ptr<rclcpp::Client<
        collision_avoidance_interfaces::srv::VelocityReciprocalsCollisionAvoidance>>
        m_velocity_client;
    rclcpp::Logger m_logger;
};
