#pragma once

#include <string>
#include <unordered_map>

#include "pad_management_cpp/I_pad_resource_manager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "smart_pad/smart_pad_tf.hpp"
#include "smart_pad/smart_pad_neighbors.hpp"
#include "smart_pad_visualization.hpp"
#include "smart_pad/smart_pad_neighbors_lock.hpp"

#include "smart_pad_interfaces/srv/lock.hpp"
#include <Eigen/Dense>

#include "pad_management_interfaces/action/pad_execute.hpp"

namespace smart_pad
{

class SmartPadResourceManager : public IPadResourceManager
{
public:
    explicit SmartPadResourceManager(pad_management_cpp::NodeInterfacesBundle node_iface_bundle)
    : m_node_iface_bundle(node_iface_bundle)
    , m_id(node_iface_bundle.parameters_interface->declare_parameter("id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
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

        m_smart_pad_neighbors = std::make_shared<SmartPadNeighbors>(
            m_pad_name,
            m_smart_pad_tf,
            node_iface_bundle.parameters_interface,
            m_logger
        );

        m_smart_pad_visualization = std::make_shared<SmartPadVisualization>(
            m_id,
            m_pad_name,
            node_iface_bundle.base_interface,
            node_iface_bundle.topics_interface,
            node_iface_bundle.clock_interface,
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
        //std::vector<std::string> other_frames = m_smart_pad_tf->get_other_smart_pads();
        //std::stringstream ss;
        //for (const auto & frame : other_frames) {
        //    ss << frame << " ";
        //}
//
        //RCLCPP_INFO(m_logger, "Other smart pads: %s", ss.str().c_str());
    }

    void 
    lock_service_callback(
        const std::shared_ptr<smart_pad_interfaces::srv::Lock::Request> request,
        std::shared_ptr<smart_pad_interfaces::srv::Lock::Response> response)
    {
        RCLCPP_INFO(m_logger, "Received lock request for pad: %s, locking: %s", 
                    request->name.c_str(), request->locking ? "true" : "false");
        std::lock_guard<std::mutex> lock(m_locking_mutex);
        if (request->locking)
        {
            if (m_currently_locked) {
                RCLCPP_WARN(m_logger, "Pad is already locked, cannot lock again");
                response->success = false;
            } else {
                RCLCPP_INFO(m_logger, "Locking pad %s", request->name.c_str());
                m_currently_locked = true;
                m_smart_pad_visualization->set_state(1); // Set to locked state
                response->success = true;
            }
        } else {
            if (m_currently_locked) {
                response->success = true;
                m_smart_pad_visualization->set_state(0); // Set to available state
                m_currently_locked = false;
            } 
            else {
                response->success = false;
            }
        }      
        
        // std::vector<std::string> neighbors;
        // if (m_smart_pad_neighbors->get_neighbors(neighbors)) {
        //     std::stringstream ss;
        //     for (const auto & neighbor : neighbors) {
        //         ss << neighbor << " ";
        //     }
        //     RCLCPP_INFO(m_logger, "Neighbors within threshold: %s", ss.str().c_str());
        // } else {
        //     RCLCPP_WARN(m_logger, "Failed to get neighbors.");
        // }
// 
        // RCLCPP_INFO(m_logger, "Received lock request for pad: %s, locking: %s", 
        //             request->name.c_str(), request->locking ? "true" : "false");
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

    std::lock_guard<std::mutex> locking_lock(m_locking_mutex);
    if (m_currently_locked) {
        return false;
    } 
    RCLCPP_INFO(m_logger, "Trying to acquire neighbors lock for pad %u", static_cast<unsigned>(id));
    std::shared_ptr<NeighborsLock> smart_pad_neighbors_lock = std::make_shared<NeighborsLock>(
        m_pad_name,
        m_smart_pad_neighbors,
        m_node_iface_bundle.base_interface,
        m_node_iface_bundle.graph_interface,
        m_node_iface_bundle.services_interface,
        m_logger
    ); 
    if (!smart_pad_neighbors_lock->try_lock()) {
        RCLCPP_WARN(m_logger, "Failed to acquire neighbors lock for pad %u", static_cast<unsigned>(id));
        return false;
    } else {
        RCLCPP_INFO(m_logger, "Successfully acquired neighbors lock for pad %u", static_cast<unsigned>(id));
        m_current_smart_pad_neighbors_lock = smart_pad_neighbors_lock;
        m_currently_locked = true;
    }

    RCLCPP_INFO(m_logger, "Lock acquired for pad %u", static_cast<unsigned>(id));
    m_locks.emplace(id, std::move(lock));
    m_smart_pad_visualization->set_state(1); // Set to locked state
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

    std::lock_guard<std::mutex> lock(m_locking_mutex);
    m_current_smart_pad_neighbors_lock.reset();
    m_currently_locked = false;

    m_locks.erase(it);

    if (result == pad_management_interfaces::action::PadExecute::Result::RESULT_ON_PAD)
    {
        m_smart_pad_visualization->set_state(2); // Set to occupied state
        RCLCPP_INFO(m_logger, "Pad %u executed successfully, setting state to occupied", static_cast<unsigned>(id));
    } else if (result == pad_management_interfaces::action::PadExecute::Result::RESULT_NOT_ON_PAD){
        m_smart_pad_visualization->set_state(0); // Set to available state
        RCLCPP_INFO(m_logger, "Pad %u released and not on pad", static_cast<unsigned>(id));
    } else {
        m_smart_pad_visualization->set_state(3); // Set to error state
        RCLCPP_WARN(m_logger, "Pad %u released with unknown result %u, setting state to error", static_cast<unsigned>(id), static_cast<unsigned>(result));

    }
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


private: 
    std::mutex m_locking_mutex;
    bool m_currently_locked = false;
    std::shared_ptr<NeighborsLock> m_current_smart_pad_neighbors_lock;


private: 
  std::unordered_map<uint8_t, std::unique_lock<std::mutex>> m_locks;
  std::mutex m_lock_mutex;



private: 
    pad_management_cpp::NodeInterfacesBundle m_node_iface_bundle;
    int m_id;
    std::string m_pad_name;
    rclcpp::Logger m_logger;
    std::shared_ptr<SmartPadTF> m_smart_pad_tf;
    std::shared_ptr<SmartPadNeighbors> m_smart_pad_neighbors;
    std::shared_ptr<SmartPadVisualization> m_smart_pad_visualization;

    
    std::shared_ptr<rclcpp::TimerBase> m_timer;

    std::shared_ptr<rclcpp::CallbackGroup> m_lock_service_callback_group;
    std::shared_ptr<rclcpp::Service<smart_pad_interfaces::srv::Lock>> m_lock_service;
};

}  // namespace smart_pad
