#pragma once
#include "padflies_cpp/commander_base.hpp"

class SimCommander : public PadflieCommanderBase
{
   

public: 
    SimCommander(
        const std::string & prefix,
        const std::string & cf_prefix,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface)
    : PadflieCommanderBase(prefix, cf_prefix, node_base_interface, node_param_interface, node_clock_interface, node_logging_interface)
    , m_logger(node_logging_interface->get_logger())
    {
    }

    bool is_healthy() const override {
        return true;
    }

    bool get_home_state() const override {
        return false; // We are never "home"
    }

private: 
    void m_handle_takeoff_command(
        const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
        const std::shared_ptr<rmw_request_id_t> request_id,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req) override {
        auto response = std_srvs::srv::Trigger::Response();
        response.success = true;
        response.message = "Takeoff command received";
        
        RCLCPP_INFO(m_logger, "Takeoff command received. Sending takeoff target.");
        // So that the target is initialized
        EigenPoseStamped target;
        target.pose = Eigen::Affine3d::Identity();
        target.pose.translation() = Eigen::Vector3d(0.0, 0.0, 2.0); // Takeoff to 2 meters height
        target.frame_id = "world";
        m_hardware_actor->set_pose_target(target, false); // Don't use yaw for takeoff


        service_handle->send_response(*request_id, response);
    }

    void m_handle_land_command(
        const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
        const std::shared_ptr<rmw_request_id_t> request_id,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req) override {
        auto response = std_srvs::srv::Trigger::Response();
        response.success = true;
        response.message = "Land command received";
        service_handle->send_response(*request_id, response);
    }

    void m_handle_send_target_command(const padflies_interfaces::msg::SendTarget::SharedPtr msg) override {
        EigenPoseStamped target;
        target.pose = Eigen::Affine3d::Identity();
        Eigen::Translation3d translation(
            msg->target.pose.position.x,
            msg->target.pose.position.y,
            msg->target.pose.position.z
        );
        Eigen::Quaterniond rotation(
            msg->target.pose.orientation.w,
            msg->target.pose.orientation.x,
            msg->target.pose.orientation.y,
            msg->target.pose.orientation.z
        );

        target.pose = translation * rotation;
        target.frame_id = msg->target.header.frame_id;
        m_hardware_actor->set_pose_target(target, msg->use_yaw);
    }

private: 
    rclcpp::Logger m_logger;

};