#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "padflies_interfaces/msg/send_target.hpp"
#include <Eigen/Dense>
namespace rqt_padflies
{
class PadflieControlConnection
{
public:
    PadflieControlConnection(
        std::string prefix,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        rclcpp::CallbackGroup::SharedPtr callback_group)
    {    
        auto pub_options = rclcpp::PublisherOptions();
        m_send_target_publisher = rclcpp::create_publisher<padflies_interfaces::msg::SendTarget>(
            node_topics_interface,
            prefix + "/send_target",
            10,
            pub_options);

        m_land_client = rclcpp::create_client<std_srvs::srv::Trigger>(
            node_base_interface,
            node_graph_interface,
            node_services_interface,
            prefix + "/land",
            rclcpp::QoS(10).get_rmw_qos_profile(),
            callback_group);

        m_takeoff_client = rclcpp::create_client<std_srvs::srv::Trigger>(
            node_base_interface,
            node_graph_interface,
            node_services_interface,
            prefix + "/takeoff",
            rclcpp::QoS(10).get_rmw_qos_profile(),
            callback_group);
    }

    void set_target(Eigen::Vector3d target)
    {
        m_current_target = target;
        auto message = padflies_interfaces::msg::SendTarget();
        message.target.header.frame_id = "world";
        message.target.pose.position.x = target.x();
        message.target.pose.position.y = target.y();
        message.target.pose.position.z = target.z();
        m_send_target_publisher->publish(message);
    }

    bool get_target(Eigen::Vector3d &target)
    {
        target = m_current_target;
        return m_has_target;
    }

    void land()
    {
        if (!m_land_client->service_is_ready()) return;
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        m_land_client->async_send_request(request);
    }

    void takeoff()
    {
        if (!m_takeoff_client->service_is_ready()) return;
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        m_takeoff_client->async_send_request(request);
    }

private:
    std::shared_ptr<rclcpp::Publisher<padflies_interfaces::msg::SendTarget>> m_send_target_publisher;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> m_land_client;
    std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> m_takeoff_client;

    bool m_has_target = false;
    Eigen::Vector3d m_current_target = Eigen::Vector3d(0.0, 0.0, 2.0);

};

} // namespace rqt_padflies