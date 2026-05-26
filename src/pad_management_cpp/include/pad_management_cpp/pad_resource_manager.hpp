#pragma once

#include <string>
#include "pad_management_cpp/I_pad_resource_manager.hpp"
#include <unordered_map>

namespace pad_management_cpp
{

class PadResourceManager : public IPadResourceManager
{
public:
explicit PadResourceManager(NodeInterfacesBundle node_interfaces_bundle) 
    {
        RCLCPP_INFO(rclcpp::get_logger("PadResourceManager"), "PadResourceManager constructor called.");
    }

    bool m_try_lock(uint8_t id) override;

    void m_release(uint8_t id, uint8_t result) override;

    bool m_get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position) override;
private: 

    std::string get_pad_name(uint8_t id) const {
        return "pad_" + std::to_string(id);
    }

    std::unordered_map<uint8_t, std::unique_lock<std::mutex>> m_locks;

    
    std::mutex m_mutex;
};

};