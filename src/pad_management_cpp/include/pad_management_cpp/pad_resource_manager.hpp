#pragma once

#include <string>
#include "pad_management_cpp/I_pad_resource_manager.hpp"
#include <unordered_map>

namespace pad_management_cpp
{

class PadResourceManager : public IPadResourceManager
{
public:
    explicit PadResourceManager(pad_management_cpp::NodeInterfacesBundle node_interfaces_bundle) 
    : m_logger(node_interfaces_bundle.logging_interface->get_logger().get_child("PadResourceManager"))
    {
        RCLCPP_INFO(m_logger, "PadResourceManager constructor called.");
    }

    bool m_try_lock(uint8_t id) override;

    void m_release(uint8_t id, uint8_t result) override;

    bool m_get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position) override;

    std::vector<std::string> get_pad_tf_names() override
    {
        // For simplicity, we assume that the pad TF names are "pad_0", "pad_1", ..., "pad_255"
        std::vector<std::string> tf_names;
        for (int i = 0; i < 256; ++i) {
            tf_names.push_back(get_pad_name(i));
        }
        return tf_names;
    }
    
    std::string get_pad_name(uint8_t id) const {
        return "pad_" + std::to_string(id);
    }

private: 
    int m_current_hodler = -1;

    rclcpp::Logger m_logger;
    std::mutex m_mutex;
};

};