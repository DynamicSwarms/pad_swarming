#pragma once

#include "rclcpp/rclcpp.hpp"

#include "crazyflie_interfaces/srv/notify_setpoints_stop.hpp"
#include "crazyflie_interfaces/msg/position.hpp"

#include <Eigen/Dense>




class LowLevelCommanderMinimal
{
public:
    LowLevelCommanderMinimal(
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,        
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
    rclcpp::Logger m_logger;


    std::shared_ptr<rclcpp::Client<crazyflie_interfaces::srv::NotifySetpointsStop>> m_notify_setpoints_stop_client;
    std::shared_ptr<rclcpp::Publisher<crazyflie_interfaces::msg::Position>> m_cmd_position_pub;
};