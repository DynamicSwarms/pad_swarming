#pragma once
#include "padflies_cpp/commander_base.hpp"
#include "padflies_cpp/node_interfaces_bundle.hpp"

class SimCommander : public PadflieCommanderBase
{
   

public: 
    SimCommander(
        const std::string & prefix,
        const std::string & cf_prefix,
        padflies_cpp::NodeInterfacesBundle node_interfaces_bundle
    )
    : PadflieCommanderBase(prefix, cf_prefix, node_interfaces_bundle)
    , m_logger(node_interfaces_bundle.logging_interface->get_logger())
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
        (void)req;
        auto response = std_srvs::srv::Trigger::Response();
        response.success = true;
        response.message = "Takeoff command received";
        
        RCLCPP_INFO(m_logger, "Takeoff command received. Sending takeoff target.");
        // So that the target is initialized
        PoseTarget target;
        target.pose = Eigen::Affine3d::Identity();
        target.pose.translation() = Eigen::Vector3d(0.0, 0.0, 2.0); // Takeoff to 2 meters height
        target.frame_id = "world";
        target.use_yaw = false;
        target.collision_avoidance = true;
        m_hardware_actor->set_pose_target(target); // Don't use yaw for takeoff


        service_handle->send_response(*request_id, response);
    }

    void m_handle_land_command(
        const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
        const std::shared_ptr<rmw_request_id_t> request_id,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> req) override {
        (void)req;
        auto response = std_srvs::srv::Trigger::Response();
        response.success = true;
        response.message = "Land command received";
        service_handle->send_response(*request_id, response);
    }

    void m_handle_send_target_command(const padflies_interfaces::msg::SendTarget::SharedPtr msg) override {
        PoseTarget target;
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
        target.use_yaw = msg->use_yaw;
        target.collision_avoidance = msg->collision_avoidance;
        m_hardware_actor->set_pose_target(target);
    }

private: 
    rclcpp::Logger m_logger;

};