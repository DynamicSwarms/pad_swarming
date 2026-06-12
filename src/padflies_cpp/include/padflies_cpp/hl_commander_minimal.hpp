#pragma once

#include "rclcpp/rclcpp.hpp"


#include "crazyflie_interfaces/srv/takeoff.hpp"
#include "crazyflie_interfaces/srv/land.hpp"
#include "crazyflie_interfaces/srv/go_to.hpp"

#include <Eigen/Dense>




class HighLevelCommanderMinimal
{
public:
    HighLevelCommanderMinimal(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
        const std::string & cf_prefix);

    ~HighLevelCommanderMinimal();

    void takeoff(
        double height,
        double duration_seconds,
        double yaw_rad,
        double group_mask = 0
    );


    void land(
        double target_height,
        double duration_seconds,
        double yaw_rad,
        double group_mask = 0
    );

    void go_to(
        const Eigen::Vector3d & position,
        double yaw_rad,
        double duration_seconds,
        bool relative = false,
        double group_mask = 0
    );

private: 
    std::string m_cf_prefix;
    rclcpp::Logger m_logger;

    std::shared_ptr<rclcpp::Client<crazyflie_interfaces::srv::Takeoff>> m_takeoff_client;
    std::shared_ptr<rclcpp::Client<crazyflie_interfaces::srv::Land>> m_land_client;
    std::shared_ptr<rclcpp::Client<crazyflie_interfaces::srv::GoTo>> m_go_to_client;
};  