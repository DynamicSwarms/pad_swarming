#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "collision_avoidance_interfaces/srv/collision_avoidance.hpp"

#include <Eigen/Dense>

class CollisionAvoidanceClient
{
public:
    CollisionAvoidanceClient(
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
        uint8_t cf_id);

    ~CollisionAvoidanceClient();

    void get_collision_avoidance_target(
        const Eigen::Vector3d & position,
        Eigen::Vector3d & target);

private: 
    uint8_t m_cf_id;
    rclcpp::Client<collision_avoidance_interfaces::srv::CollisionAvoidance>::SharedPtr m_client;
    std::string m_logger_name;
};