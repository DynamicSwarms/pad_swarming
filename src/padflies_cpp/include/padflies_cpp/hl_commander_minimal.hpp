#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "crazyflie_interfaces/msg/takeoff.hpp"
#include "crazyflie_interfaces/msg/land.hpp"
#include "crazyflie_interfaces/msg/go_to.hpp"

#include <Eigen/Dense>




class HighLevelCommanderMinimal
{
public:
    HighLevelCommanderMinimal(
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
        const std::string & cf_prefix);

    ~HighLevelCommanderMinimal();

    void takeoff(
        double height,
        double duration_seconds,
        double yaw,
        bool use_current_yaw = false, 
        double group_mask = 0
    );


    void land(
        double target_height,
        double duration_seconds,
        double yaw,
        bool use_current_yaw = false,
        double group_mask = 0
    );

    void go_to(
        const Eigen::Vector3d & position,
        double yaw,
        double duration_seconds,
        bool relative = false,
        bool linear = false,
        double group_mask = 0
    );

private: 
    std::string m_cf_prefix;
    
    rclcpp::Publisher<crazyflie_interfaces::msg::Takeoff>::SharedPtr m_takeoff_pub;
    rclcpp::Publisher<crazyflie_interfaces::msg::Land>::SharedPtr m_land_pub;
    rclcpp::Publisher<crazyflie_interfaces::msg::GoTo>::SharedPtr m_go_to_pub;

};