#pragma once

#include "pad_management_cpp/I_pad_resource_manager.hpp"
#include <unordered_map>

class PadResourceManager : public IPadResourceManager
{
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