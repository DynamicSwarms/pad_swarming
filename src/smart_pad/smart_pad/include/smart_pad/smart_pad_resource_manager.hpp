#pragma once

#include <string>
#include <unordered_map>

#include "pad_management_cpp/I_pad_resource_manager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "smart_pad/smart_pad_tf.hpp"
#include "smart_pad/smart_pad_neighbors.hpp"
#include "smart_pad_interfaces/srv/lock.hpp"
#include <Eigen/Dense>

namespace smart_pad
{

class SmartPadResourceManager : public IPadResourceManager
{
public:
    explicit SmartPadResourceManager(pad_management_cpp::NodeInterfacesBundle node_iface_bundle)
    : m_id(node_iface_bundle.parameters_interface->declare_parameter("id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , m_pad_name("smart_pad_" + std::to_string(m_id))
    , m_logger(node_iface_bundle.logging_interface->get_logger())
    {
        m_smart_pad_tf = std::make_shared<SmartPadTF>(
            m_pad_name,
            node_iface_bundle.base_interface,
            node_iface_bundle.topics_interface,
            node_iface_bundle.clock_interface,
            m_logger
        );

        m_smart_pad_neighbors = std::make_unique<SmartPadNeighbors>(
            m_pad_name,
            m_smart_pad_tf,
            node_iface_bundle.parameters_interface,
            m_logger
        );

        m_timer = rclcpp::create_timer(
            node_iface_bundle.base_interface,
            node_iface_bundle.timers_interface,
            node_iface_bundle.clock_interface->get_clock(),
            std::chrono::milliseconds(1000), 
            std::bind(&SmartPadResourceManager::timer_callback, this)
        );

        m_lock_service_callback_group = node_iface_bundle.base_interface->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        m_lock_service = rclcpp::create_service<smart_pad_interfaces::srv::Lock>(
            node_iface_bundle.base_interface,
            node_iface_bundle.services_interface,
            "~/lock",
            std::bind(&SmartPadResourceManager::lock_service_callback, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(), 
            m_lock_service_callback_group
        );

    }

    void
    timer_callback() {
        std::vector<std::string> other_frames = m_smart_pad_tf->get_other_smart_pads();
        std::stringstream ss;
        for (const auto & frame : other_frames) {
            ss << frame << " ";
        }
        RCLCPP_INFO(m_logger, "Other smart pads: %s", ss.str().c_str());
    }

    void 
    lock_service_callback(
        const std::shared_ptr<smart_pad_interfaces::srv::Lock::Request> request,
        std::shared_ptr<smart_pad_interfaces::srv::Lock::Response> response)
    {
        std::vector<std::string> neighbors;
        if (m_smart_pad_neighbors->get_neighbors(neighbors)) {
            std::stringstream ss;
            for (const auto & neighbor : neighbors) {
                ss << neighbor << " ";
            }
            RCLCPP_INFO(m_logger, "Neighbors within threshold: %s", ss.str().c_str());
        } else {
            RCLCPP_WARN(m_logger, "Failed to get neighbors.");
        }

        RCLCPP_INFO(m_logger, "Received lock request for pad: %s, locking: %s", 
                    request->name.c_str(), request->locking ? "true" : "false");
    }

private:
  bool m_try_lock(uint8_t id) override
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

  void m_release(uint8_t id, uint8_t result) override
  {
    (void)result;
    auto it = m_locks.find(id);
    if (it == m_locks.end()) {
        RCLCPP_WARN(m_logger, "Release requested for unlocked pad %u", static_cast<unsigned>(id));
        return;
    }

    m_locks.erase(it);
  }

  bool m_get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position) override
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

  std::string get_pad_name(uint8_t id) const {return "smart_pad_" + std::to_string(id);}

  std::unordered_map<uint8_t, std::unique_lock<std::mutex>> m_locks;
  std::mutex m_lock_mutex;

private: 
    int m_id;
    std::string m_pad_name;
    rclcpp::Logger m_logger;
    std::shared_ptr<SmartPadTF> m_smart_pad_tf;
    std::unique_ptr<SmartPadNeighbors> m_smart_pad_neighbors;

    std::shared_ptr<rclcpp::TimerBase> m_timer;

    std::shared_ptr<rclcpp::CallbackGroup> m_lock_service_callback_group;
    std::shared_ptr<rclcpp::Service<smart_pad_interfaces::srv::Lock>> m_lock_service;
};

}  // namespace smart_pad
