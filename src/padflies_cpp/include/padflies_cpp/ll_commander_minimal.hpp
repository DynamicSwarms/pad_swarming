#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "crazyflie_interfaces/msg/notify_setpoints_stop.hpp"
#include "crazyflie_interfaces/msg/position.hpp"

#include <Eigen/Dense>




class LowLevelCommanderMinimal
{
public:
    LowLevelCommanderMinimal(
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
        const std::string & cf_prefix);

    ~LowLevelCommanderMinimal();

    void notify_setpoints_stop(
        int remain_valid_milliseconds = 100, 
        double group_mask = 0
    );


    void cmd_position(
        const Eigen::Vector3d & position,
        double yaw
    );

private: 
    std::string m_cf_prefix;

    rclcpp::Publisher<crazyflie_interfaces::msg::NotifySetpointsStop>::SharedPtr m_notify_setpoints_stop_pub;
    rclcpp::Publisher<crazyflie_interfaces::msg::Position>::SharedPtr m_cmd_position_pub;

    std::string m_logger_name;
};