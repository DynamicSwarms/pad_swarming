#include "smart_pad/smart_pad_resource_manager.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace smart_pad
{

bool SmartPadResourceManager::m_try_lock(uint8_t id)
{
  if (m_locks.find(id) != m_locks.end()) {
    RCLCPP_WARN(m_logger, "Pad %u already has a lock entry", static_cast<unsigned>(id));
    return false;
  }

  std::unique_lock<std::mutex> lock(m_lock_mutex, std::defer_lock);
  if (!lock.try_lock()) {
    return false;
  }

  m_locks.emplace(id, std::move(lock));
  return true;
}

void SmartPadResourceManager::m_release(uint8_t id, uint8_t result)
{
  (void)result;
  auto it = m_locks.find(id);
  if (it == m_locks.end()) {
    RCLCPP_WARN(m_logger, "Release requested for unlocked pad %u", static_cast<unsigned>(id));
    return;
  }

  m_locks.erase(it);
}

bool SmartPadResourceManager::m_get_associated_position(
  uint8_t id,
  geometry_msgs::msg::PoseStamped & position)
{
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

std::string SmartPadResourceManager::get_pad_name(uint8_t id) const
{
  return "smart_pad_" + std::to_string(id);
}

}  // namespace smart_pad

PLUGINLIB_EXPORT_CLASS(smart_pad::SmartPadResourceManager, IPadResourceManager)
