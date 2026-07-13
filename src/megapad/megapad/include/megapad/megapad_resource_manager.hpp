#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pad_management_cpp/I_pad_resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace megapad
{
using namespace pad_management_cpp;

class MegaPadResourceManager : public IPadResourceManager
{
public:
    explicit MegaPadResourceManager(NodeInterfacesBundle node_interfaces_bundle)
    : m_logger(node_interfaces_bundle.logging_interface->get_logger().get_child("MegaPadResourceManager"))
    , m_clock_interface(node_interfaces_bundle.clock_interface)
    {
        double max_hold_time_sec = node_interfaces_bundle.parameters_interface->declare_parameter("max_hold_time", rclcpp::ParameterValue(40.0)).get<double>();
        p_max_hold_time = rclcpp::Duration::from_seconds(max_hold_time_sec);

        node_interfaces_bundle.parameters_interface->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & parameters) {
                for (const auto & param : parameters) {
                    if (param.get_name() == "max_hold_time") {
                        p_max_hold_time = rclcpp::Duration::from_seconds(param.as_double());
                        RCLCPP_INFO(m_logger, "Updated max_hold_time to %f seconds", param.as_double());
                    }
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                result.reason = "";
                return result;
            }
        );

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

public:
    AccessHandle submit_access_request(const AccessRequest & request) override
    {
        m_access_requests[request.id] = request;
        return AccessHandle{request.id};
    }

    AccessResponse query_request_status(const AccessHandle & handle) override
    {
        const std::lock_guard<std::mutex> lock(m_holder_mutex);
        if (m_current_holder == -1)
        {
            std::vector<uint8_t> ordered_ids = get_ordered_request_ids();
            if (!ordered_ids.empty() && ordered_ids[0] == handle.id)
            {
                m_current_holder = handle.id;
                m_current_holder_start_time = m_clock_interface->get_clock()->now();
                RCLCPP_INFO(m_logger, "Lock acquired for cf %d", handle.id);
                return AccessResponse{AccessResponse::Result::ACCEPTED, "Accepted", p_max_hold_time};
            } else  {
                std::ostringstream oss;
                for (size_t i = 0; i < ordered_ids.size(); ++i) {
                    if (i > 0) {
                        oss << ", ";
                    }
                    oss << static_cast<int>(ordered_ids[i]);
                }
                RCLCPP_INFO(m_logger, "Lock not available for cf %d, current holder: %d and sorting: %s", handle.id, m_current_holder, oss.str().c_str());
            }
        }

        rclcpp::Duration wait_time = estimate_wait_time(handle.id);

        return AccessResponse{AccessResponse::Result::PENDING, "Pending", wait_time};
    }

    void notify_update(const AccessHandle & handle, const ExecuteUpdate & update) override
    {
        RCLCPP_INFO(m_logger, "Received update for cf %d: status %d", handle.id, update.status);
    }

    void notify_finished(const AccessHandle & handle, const ExecuteResult & result) override
    {
        const std::lock_guard<std::mutex> lock(m_holder_mutex);
        RCLCPP_INFO(m_logger, "Execution finished for cf %d with result %d", handle.id, result.result);
        if (m_current_holder == handle.id) {
            m_current_holder = -1;
            RCLCPP_INFO(m_logger, "Lock released for cf %d", handle.id);
        }
        m_access_requests.erase(handle.id);
    }

    void cancel(const AccessHandle & handle) override
    {
        const std::lock_guard<std::mutex> lock(m_holder_mutex);
        RCLCPP_INFO(m_logger, "Cancel requested for cf %d", handle.id);
        if (m_current_holder == handle.id) {
            m_current_holder = -1;
            RCLCPP_INFO(m_logger, "Lock released for cf %d due to cancel", handle.id);
        }
        m_access_requests.erase(handle.id);
    }

    bool get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position) override
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
private: 
    rclcpp::Duration estimate_wait_time(uint8_t id)
    {
        if (m_access_requests.find(id) == m_access_requests.end()) {
            RCLCPP_WARN(m_logger, "Access request for cf %d not found", id);
            return rclcpp::Duration::from_seconds(0);            
        }

        std::vector<uint8_t> ordered_ids = get_ordered_request_ids();

        auto now = m_clock_interface->get_clock()->now();
        rclcpp::Duration wait_time = rclcpp::Duration::from_seconds(0);
        for (const auto ordered_id : ordered_ids)
        {
            if (ordered_id == id) break;

            if (m_access_requests.find(ordered_id) != m_access_requests.end()) {
                const auto & req = m_access_requests[ordered_id];
                if (m_current_holder == ordered_id) {
                    rclcpp::Duration time_held = now - m_current_holder_start_time;
                    rclcpp::Duration remaining_time = req.usage_time - time_held;
                    rclcpp::Duration time_left = remaining_time;
                    wait_time += time_left;
                } else {
                    wait_time += req.usage_time;
                }
            }
        }
        return wait_time;
    }

    std::vector<uint8_t> get_ordered_request_ids() const // sorts by id
    {
        return this->sort_by_id();
    }

    std::vector<uint8_t> sort_by_id() const
    {
        std::vector<uint8_t> ordered_ids;
        for (const auto & pair : m_access_requests) {
            ordered_ids.push_back(pair.first);
        }
        std::sort(ordered_ids.begin(), ordered_ids.end());
        return ordered_ids;
    }

    std::vector<uint8_t> sort_by_request_time() const
    {
        std::vector<std::pair<uint8_t, rclcpp::Time>> id_time_pairs;
        for (const auto & pair : m_access_requests) {
            id_time_pairs.emplace_back(pair.first, pair.second.request_time);
        }
        std::sort(id_time_pairs.begin(), id_time_pairs.end(),
                  [](const auto & a, const auto & b) { return a.second < b.second; });

        std::vector<uint8_t> ordered_ids;
        for (const auto & pair : id_time_pairs) {
            ordered_ids.push_back(pair.first);
        }
        return ordered_ids;
    }

    std::vector<uint8_t> sort_by_battery_percentage() const
    {
        std::vector<std::pair<uint8_t, double>> id_battery_pairs;
        for (const auto & pair : m_access_requests) {
            id_battery_pairs.emplace_back(pair.first, pair.second.battery_percentage);
        }
        std::sort(id_battery_pairs.begin(), id_battery_pairs.end(),
                  [](const auto & a, const auto & b) { return a.second > b.second; });

        std::vector<uint8_t> ordered_ids;
        for (const auto & pair : id_battery_pairs) {
            ordered_ids.push_back(pair.first);
        }
        return ordered_ids;
    }

private:
    std::mutex m_holder_mutex;
    int m_current_holder = -1;
    rclcpp::Time m_current_holder_start_time;

    rclcpp::Duration p_max_hold_time = rclcpp::Duration::from_seconds(0.0);

    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_clock_interface;

    std::unordered_map<uint8_t, AccessRequest> m_access_requests;
};

}  // namespace megapad
