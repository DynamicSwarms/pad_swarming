#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "pad_management_cpp/I_pad_resource_manager.hpp"

#include "rclcpp/rclcpp.hpp"
#include "smart_pad/smart_pad_tf.hpp"
#include "smart_pad/smart_pad_neighbors.hpp"
#include "smart_pad_visualization.hpp"
#include "smart_pad/smart_pad_neighbors_lock.hpp"
#include "smart_pad/smart_pad_pad_idle_target_service.hpp"

#include "smart_pad_interfaces/srv/lock.hpp"
#include <Eigen/Dense>

#include "pad_management_interfaces/action/pad_execute.hpp"
#include "pad_management_interfaces/action/pad_right_control.hpp"

namespace smart_pad
{
using namespace pad_management_cpp;


class SmartPadResourceManager : public IPadResourceManager
{
public:
    explicit SmartPadResourceManager(pad_management_cpp::NodeInterfacesBundle node_iface_bundle)
    : m_node_iface_bundle(node_iface_bundle)
    , m_id(node_iface_bundle.parameters_interface->declare_parameter("id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
    , p_allow_takeoffs(node_iface_bundle.parameters_interface->declare_parameter("allow_takeoffs", rclcpp::ParameterValue(true)).get<bool>())
    , p_allow_landings(node_iface_bundle.parameters_interface->declare_parameter("allow_landings", rclcpp::ParameterValue(true)).get<bool>())
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
            m_lock_service_callback_group);

        m_pad_idle_target_service = std::make_shared<SmartPadPadIdleTargetService>(
            node_iface_bundle.base_interface,
            node_iface_bundle.services_interface,
            m_smart_pad_tf,
            m_logger
        );

        m_param_callback_handle = node_iface_bundle.parameters_interface->add_on_set_parameters_callback(
            std::bind(&SmartPadResourceManager::m_on_parameters_set, this, std::placeholders::_1));

    }

    void
    timer_callback() {
    }

    void 
    lock_service_callback(
        const std::shared_ptr<smart_pad_interfaces::srv::Lock::Request> request,
        std::shared_ptr<smart_pad_interfaces::srv::Lock::Response> response)
    {
        RCLCPP_INFO(m_logger, "Received lock request for pad: %s, locking: %s", 
                    request->name.c_str(), request->locking ? "true" : "false");
        std::lock_guard<std::mutex> lock(m_neighbors_locking_change_mutex);
        
        if (request->locking)
        {
            if (m_current_usage_lock && m_current_usage_lock.owns_lock()) {
                RCLCPP_WARN(m_logger, "Pad is locked by landing/takeoff procedure by cf %u, cannot lock", static_cast<unsigned>(m_current_usage_lock_user_id));
                response->success = false;
            }

            m_locked_by_list.push_back(request->name);
            m_locked_by_neighbors = true;
            response->success = true;

            RCLCPP_INFO(m_logger, "Locking pad %s, currently locked by neighbors: %s", request->name.c_str(), m_locked_by_neighbors ? "true" : "false");
        } else { // Unlock request
            bool removed_lock_holder = false;
            for (auto it = m_locked_by_list.begin(); it != m_locked_by_list.end(); ++it) {
                if (*it == request->name) {
                    m_locked_by_list.erase(it);
                    removed_lock_holder = true;
                    break;
                }
            }
            response->success = removed_lock_holder;
        }  

        if (m_locked_by_list.empty()) {
            m_locked_by_neighbors = false;
            m_smart_pad_visualization->set_state(m_currently_usage_locked ? SmartPadVisualization::VisualizationState::NEIGHBOR_LOCKED : SmartPadVisualization::VisualizationState::AVAILABLE); // Set state dependent if currently used
        }
        
        set_availability();
        update_visualization();
    }

private:
  AccessHandle submit_access_request(const AccessRequest & request) override
  {
    m_access_requests[request.id] = request;
    return AccessHandle{request.id};
  }

  AccessResponse query_request_status(const AccessHandle & handle) override
  {
    uint8_t id = handle.id;
    auto it = m_access_requests.find(handle.id);
    if (it == m_access_requests.end())
    {
      RCLCPP_WARN(m_logger, "Access request with id %u not found", static_cast<unsigned>(handle.id));
      return AccessResponse{AccessResponse::Result::REJECTED, "Request not found", rclcpp::Duration(0, 0)};
    }

    if (m_currently_usage_locked && m_current_usage_lock_user_id != handle.id)
    {
      RCLCPP_WARN(m_logger, "Access request with id %u is not the current usage lock holder", static_cast<unsigned>(handle.id));
      return AccessResponse{AccessResponse::Result::REJECTED, "Not the current usage lock holder", rclcpp::Duration(0, 0)};
    }

    
    // We either are the current usage lock holder or the lock is not currently held by anyone.

    uint8_t action = it->second.action;
    if ((action == pad_management_interfaces::action::PadRightControl::Goal::ACTION_TAKEOFF && !p_allow_takeoffs ||
         action == pad_management_interfaces::action::PadRightControl::Goal::ACTION_LAND && !p_allow_landings)) {
        RCLCPP_WARN(m_logger, "Lock request for cf %u with action %u denied due to configuration (allow_takeoffs: %s, allow_landings: %s)", 
                    static_cast<unsigned>(handle.id), 
                    static_cast<unsigned>(action),
                    p_allow_takeoffs ? "true" : "false",
                    p_allow_landings ? "true" : "false");

        if (m_currently_usage_locked && m_current_usage_lock_user_id == handle.id)
        {
            // We are the holder, but configuration changed -> need to release the lock.
            m_current_usage_lock.unlock();
            m_currently_usage_locked = false;
            RCLCPP_INFO(m_logger, "Usage lock released for cf %u due to configuration denial", static_cast<unsigned>(handle.id));
        }
        return AccessResponse{AccessResponse::Result::REJECTED, "Action not allowed by configuration", rclcpp::Duration(0, 0)};
    }

    // The configuration allows the action, we now need to check if we are holder 
    if (!m_currently_usage_locked)
    {
        m_current_usage_lock = std::unique_lock<std::mutex>(m_current_usage_lock_mutex, std::defer_lock);
        if (m_current_usage_lock.try_lock())
        {
            m_current_usage_lock_user_id = id;
            m_currently_usage_locked = true;
            m_current_usage_action = 
                (action == pad_management_interfaces::action::PadRightControl::Goal::ACTION_TAKEOFF) ? UsageAction::TAKEOFF : UsageAction::LANDING;
        } else {
            RCLCPP_WARN(m_logger, "Failed to acquire usage lock for cf %u", static_cast<unsigned>(handle.id));
            return AccessResponse{AccessResponse::Result::REJECTED, "Failed to acquire usage lock.", rclcpp::Duration(0, 0)};
        }
        RCLCPP_INFO(m_logger, "Lock acquired for cf %u", static_cast<unsigned>(handle.id));
    }

    // We are now the holder of the usage lock, we can now check if we can acquire the neighbors lock.

    std::lock_guard<std::mutex> neighbors_change_lock(m_neighbors_locking_change_mutex);
    if (m_locked_by_neighbors)
    {
        RCLCPP_WARN(m_logger, "Cannot acquire lock for cf %u because pad is locked by neighbors", static_cast<unsigned>(id));
        return AccessResponse{AccessResponse::Result::PENDING, "Pad is locked by neighbors", rclcpp::Duration(0, 0)};
    } 

    RCLCPP_INFO(m_logger, "Trying to acquire neighbors lock for cf %u", static_cast<unsigned>(id));
    std::shared_ptr<NeighborsLock> smart_pad_neighbors_lock = std::make_shared<NeighborsLock>(
        m_pad_name,
        m_smart_pad_neighbors,
        m_node_iface_bundle.base_interface,
        m_node_iface_bundle.graph_interface,
        m_node_iface_bundle.services_interface,
        m_logger
    ); 
    if (!smart_pad_neighbors_lock->try_lock()) {
        RCLCPP_WARN(m_logger, "Failed to acquire neighbors lock for cf %u", static_cast<unsigned>(id));
        return AccessResponse{AccessResponse::Result::PENDING, "Failed to acquire neighbors lock", rclcpp::Duration(0, 0)};
    } 
    RCLCPP_INFO(m_logger, "Successfully acquired neighbors lock for cf %u", static_cast<unsigned>(id));  
    m_current_smart_pad_neighbors_lock = smart_pad_neighbors_lock;

    update_visualization();
    set_availability();

    return AccessResponse{AccessResponse::Result::ACCEPTED, "Accepted. Got Neighbors Lock and Usage Lock", rclcpp::Duration(40, 0)};
  }

void notify_update(const AccessHandle & handle, const ExecuteUpdate & update) override
{
    if (m_current_usage_lock_user_id == handle.id) {
        m_access_requests[handle.id].current_pose = update.current_pose;
        m_access_requests[handle.id].battery_percentage = update.battery_percentage;
        // update.status
    }
}

void notify_finished(const AccessHandle & handle, const ExecuteResult & result) override
{
    RCLCPP_INFO(m_logger, "Execution finished for cf %u with result %u", static_cast<unsigned>(handle.id), static_cast<unsigned>(result.result));
    m_free(handle);
    if (result.result == pad_management_interfaces::action::PadExecute::Result::RESULT_ON_PAD)
    {
        m_pad_state = PadState::OCCUPIED;
        RCLCPP_INFO(m_logger, "Pad %u executed successfully, setting state to occupied", static_cast<unsigned>(handle.id));
    } else if (result.result == pad_management_interfaces::action::PadExecute::Result::RESULT_NOT_ON_PAD){
        m_pad_state = PadState::AVAILABLE;
        RCLCPP_INFO(m_logger, "Pad %u released and not on pad", static_cast<unsigned>(handle.id));
    } else {
        m_pad_state = PadState::ERROR;
        RCLCPP_WARN(m_logger, "Pad %u released with unknown result %u, setting state to error", static_cast<unsigned>(handle.id), static_cast<unsigned>(result.result));
    }

        update_visualization();
        set_availability();
}

void cancel(const AccessHandle & handle) override
{
    RCLCPP_INFO(m_logger, "Received cancel request for cf %u", static_cast<unsigned>(handle.id));
    m_free(handle);
}


private: 

void m_free(const AccessHandle & handle)
{
    if (m_current_usage_lock_user_id != handle.id) {
        RCLCPP_WARN(m_logger, "Received finished notification for cf %u, but we are not the current usage lock holder (current holder: %u)", static_cast<unsigned>(handle.id), static_cast<unsigned>(m_current_usage_lock_user_id));
        return;
    }
    m_current_usage_lock.unlock();
    m_currently_usage_locked = false;
    RCLCPP_INFO(m_logger, "Usage lock released for cf %u", static_cast<unsigned>(handle.id));
    set_availability();
    m_current_smart_pad_neighbors_lock.reset();
    m_access_requests.erase(handle.id);
}

void update_visualization() {
    if (m_locked_by_neighbors && m_pad_state == PadState::OCCUPIED) {
        m_smart_pad_visualization->set_state(SmartPadVisualization::VisualizationState::NEIGHBOR_LOCKED_AND_OCCUPIED);
    } else if (m_locked_by_neighbors) {
        m_smart_pad_visualization->set_state(SmartPadVisualization::VisualizationState::NEIGHBOR_LOCKED);
    } else if (m_pad_state == PadState::OCCUPIED) {
        m_smart_pad_visualization->set_state(SmartPadVisualization::VisualizationState::OCCUPIED);
    } else if (m_pad_state == PadState::AVAILABLE) {
        m_smart_pad_visualization->set_state(SmartPadVisualization::VisualizationState::AVAILABLE);
    } else {
        m_smart_pad_visualization->set_state(SmartPadVisualization::VisualizationState::ERROR);
    }
}

void set_availability() 
{
    AvailabilityStatus status;
    status.charging_speed = AvailabilityStatus::ChargingSpeed::FAST;
    
    if (m_pad_state == PadState::OCCUPIED ||
        m_pad_state == PadState::ERROR ||
        (m_currently_usage_locked && m_current_usage_action == UsageAction::LANDING) ||
        p_allow_landings == false)
    {
        RCLCPP_INFO(m_logger, "Availability: Pad %s is not available. Occupied: %s, Error: %s, Locked by neighbors: %s, Currently usage locked: %s", 
                    m_pad_name.c_str(), 
                    m_pad_state == PadState::OCCUPIED ? "true" : "false", 
                    m_pad_state == PadState::ERROR ? "true" : "false", 
                    m_locked_by_neighbors ? "true" : "false", 
                    m_currently_usage_locked ? "true" : "false");

        
        status.available = false;
    } else {
        //         m_locked_by_neighbors || 
        status.available = true;
        if (m_locked_by_neighbors) status.wait_time = rclcpp::Duration::from_seconds(5.0 * m_locked_by_list.size());
        else status.wait_time = rclcpp::Duration::from_seconds(0.0);
    }

    update_availability(status);
}


bool get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position) override
{
    position.header.frame_id = get_pad_name(m_id);
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

std::vector<std::string> get_pad_tf_names() override
{
    return {"smart_pad_" + std::to_string(m_id)};
}


rcl_interfaces::msg::SetParametersResult m_on_parameters_set(const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto & param : parameters) {
        if (param.get_name() == "allow_takeoffs") {
            p_allow_takeoffs = param.as_bool();
            set_availability();
        } else if (param.get_name() == "allow_landings") {
            p_allow_landings = param.as_bool();
            set_availability(); 
        }
    }
    return result;
}

private: 
    std::mutex m_neighbors_locking_change_mutex;
    bool m_locked_by_neighbors = false;
    std::vector<std::string> m_locked_by_list;

private: 
    std::unordered_map<uint8_t, AccessRequest> m_access_requests;

    std::mutex m_current_usage_lock_mutex;

    std::unique_lock<std::mutex> m_current_usage_lock;
    bool m_currently_usage_locked = false;
    int m_current_usage_lock_user_id;
    enum class UsageAction {
        NONE = 0,
        TAKEOFF = 1,
        LANDING = 2
    };
    UsageAction m_current_usage_action = UsageAction::NONE;

    std::shared_ptr<NeighborsLock> m_current_smart_pad_neighbors_lock;

    enum class PadState {
        AVAILABLE = 0,
        OCCUPIED = 1,
        ERROR = 2
    };

    PadState m_pad_state = PadState::AVAILABLE;


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

    std::shared_ptr<SmartPadPadIdleTargetService> m_pad_idle_target_service;

    bool p_allow_takeoffs;
    bool p_allow_landings;
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> m_param_callback_handle; 
};

}  // namespace smart_pad
