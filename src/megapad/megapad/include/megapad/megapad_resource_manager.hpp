#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pad_management_cpp/I_pad_resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace megapad
{

class MegaPadResourceManager : public IPadResourceManager
{
public:
    explicit MegaPadResourceManager(pad_management_cpp::NodeInterfacesBundle node_interfaces_bundle)
    : m_logger(node_interfaces_bundle.logging_interface->get_logger().get_child("MegaPadResourceManager"))
    {
        RCLCPP_INFO(m_logger, "MegaPadResourceManager constructor called.");
    }

    std::vector<std::string> get_pad_tf_names() override
    {
        std::vector<std::string> tf_names;
        tf_names.reserve(256);
        for (int i = 0; i < 256; ++i) {
            tf_names.push_back(get_pad_name(i));
        }
        return tf_names;
    }

private:
    bool m_try_lock(uint8_t id, uint8_t action) override
    {
        (void)action;
        if (m_current_holder == -1) {
            m_current_holder = id;
            RCLCPP_INFO(m_logger, "Lock acquired for cf %d", id);
            return true;
        }

        RCLCPP_INFO(
            m_logger,
            "Lock already held by cf %d, cannot acquire for cf %d",
            m_current_holder,
            id);
        return false;
    }

    void m_release(uint8_t id, uint8_t result) override
    {
        if (m_current_holder != id) {
            RCLCPP_WARN(
                m_logger,
                "Release requested for cf %d, but lock is held by cf %d",
                id,
                m_current_holder);
            return;
        }

        m_current_holder = -1;
        RCLCPP_INFO(m_logger, "Lock released for cf %d with result %d", id, result);
    }

    bool m_get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position) override
    {
        RCLCPP_INFO(m_logger, "Getting associated position for cf %d", id);
        position.header.frame_id = get_pad_name(id);
        position.pose.position.x = 0.0;
        position.pose.position.y = 0.0;
        position.pose.position.z = 0.0;
        position.pose.orientation.x = 0.0;
        position.pose.orientation.y = 0.0;
        position.pose.orientation.z = 0.0;
        position.pose.orientation.w = 1.0;
        return true;
    }

    std::string get_pad_name(uint8_t id) const
    {
        return "pad_" + std::to_string(id);
    }

    int m_current_holder = -1;
    rclcpp::Logger m_logger;
};

}  // namespace megapad
