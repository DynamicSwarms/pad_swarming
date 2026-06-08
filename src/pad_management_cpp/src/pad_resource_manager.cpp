#include "pad_management_cpp/pad_resource_manager.hpp"


#include <iostream>
#include <ostream>

#include "pluginlib/class_list_macros.hpp"


namespace pad_management_cpp
{

bool 
PadResourceManager::m_try_lock(uint8_t id)
{
    if (m_current_hodler == -1) {
        m_current_hodler = id;
        RCLCPP_INFO(m_logger, "Lock acquired for cf %d", id);
        return true;
    } else {
        RCLCPP_INFO(m_logger, "Lock already held by cf %d, cannot acquire for cf %d", m_current_hodler, id);
        return false;
    }
    // RCLCPP_INFO(m_logger, "Trying to acquire lock for cf %d", id);
    // if (m_locks.find(id) != m_locks.end()) {
    //     std::cerr << "should not happen!!! only call once per name" << std::endl;
    //     return false;
    // }
    // std::unique_lock<std::mutex> lock(m_mutex, std::defer_lock);
    // if (lock.try_lock()) {
    //     m_locks.emplace(id, std::move(lock));
    //     return true;
    // } else {
    //     return false;
    // }
}

void 
PadResourceManager::m_release(uint8_t id, uint8_t result)
{
    if (m_current_hodler != id) {
        RCLCPP_WARN(m_logger, "Release requested for cf %d, but lock is held by cf %d", id, m_current_hodler);
        return;
    }
    m_current_hodler = -1;
    RCLCPP_INFO(m_logger, "Lock released for cf %d with result %d", id, result);
    // auto it = m_locks.find(id);
    // if (it != m_locks.end()) {
    //     m_locks.erase(it); // Should release
    // } else {
    //     std::cerr << "should not happen!!! trying to release unowned lock" << std::endl;
    // }
}

bool 
PadResourceManager::m_get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position)
{
    // Fixed position - id lookup
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
}  // namespace pad_management_cpp

PLUGINLIB_EXPORT_CLASS(pad_management_cpp::PadResourceManager, IPadResourceManager)