#pragma once

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "megapad/megapad_access_policy.hpp"
#include "megapad/megapad_tf.hpp"
#include "pad_management_cpp/I_pad_resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"

namespace megapad
{
using namespace pad_management_cpp;

class MegaPadResourceManagerV2 : public IPadResourceManager
{
public:
    explicit MegaPadResourceManagerV2(NodeInterfacesBundle node_interfaces_bundle)
    : m_logger(node_interfaces_bundle.logging_interface->get_logger().get_child("MegaPadResourceManagerV2"))
    , m_clock_interface(node_interfaces_bundle.clock_interface)
    , m_megapad_tf(std::make_shared<MegaPadTF>(node_interfaces_bundle, m_logger))
    , m_access_policy(std::make_unique<MegaPadAccessPolicy>())
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

        RCLCPP_INFO(m_logger, "MegaPadResourceManagerV2 constructor called.");

        pad_management_cpp::AvailabilityStatus status;
        status.charging_speed = pad_management_cpp::AvailabilityStatus::ChargingSpeed::SLOW;
        status.available = true;
        status.wait_time = rclcpp::Duration::from_seconds(0.0);
        update_availability(status);
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
        const std::lock_guard<std::mutex> lock(m_holder_mutex);
        m_access_requests[request.id] = request;
        return AccessHandle{request.id};
    }

    AccessResponse query_request_status(const AccessHandle & handle) override
    {
        const std::lock_guard<std::mutex> lock(m_holder_mutex);
        if (is_current_holder(handle.id)) {
            return AccessResponse{AccessResponse::Result::ACCEPTED, "Accepted", p_max_hold_time};
        }

        const auto request_it = m_access_requests.find(handle.id);
        if (request_it == m_access_requests.end()) {
            return AccessResponse{
                AccessResponse::Result::REJECTED, "Unknown access request",
                rclcpp::Duration::from_seconds(0.0)};
        }

        if (can_grant_access(request_it->second))
        {
            m_current_holders.push_back(CurrentHolder{
                handle.id, m_clock_interface->get_clock()->now()});
            RCLCPP_DEBUG(
                m_logger, "Access acquired for cf %u; active holders: %zu",
                static_cast<unsigned>(handle.id), m_current_holders.size());
            send_availability_update();
            return AccessResponse{
                AccessResponse::Result::ACCEPTED, "Accepted", p_max_hold_time};
        }

        rclcpp::Duration wait_time = estimate_wait_time(handle.id);

        return AccessResponse{AccessResponse::Result::PENDING, "Pending", wait_time};
    }

    void send_availability_update()
    {
        AvailabilityStatus status;
        status.available = true; //megapad always available
        status.charging_speed = AvailabilityStatus::ChargingSpeed::SLOW; //megapad always slow charging
        status.wait_time = rclcpp::Duration::from_seconds(m_access_requests.size() * 2.0);
        update_availability(status);
    }

    void notify_update(const AccessHandle & handle, const ExecuteUpdate & update) override
    {
        const std::lock_guard<std::mutex> lock(m_holder_mutex);
        const auto request_it = m_access_requests.find(handle.id);
        if (request_it == m_access_requests.end()) {
            RCLCPP_WARN(
                m_logger, "Ignoring update for unknown access request cf %u",
                static_cast<unsigned>(handle.id));
            return;
        }

        request_it->second.battery_percentage = update.battery_percentage;
        request_it->second.current_pose = update.current_pose;

        RCLCPP_DEBUG(m_logger, "Updated access request for cf %u: battery %.2f%%, position (%f, %f, %f)",
            static_cast<unsigned>(handle.id), 
            update.battery_percentage,
            update.current_pose.pose.position.x, 
            update.current_pose.pose.position.y, 
            update.current_pose.pose.position.z);
    }

    void notify_finished(const AccessHandle & handle, const ExecuteResult & result) override
    {
        (void)result;
        const std::lock_guard<std::mutex> lock(m_holder_mutex);
        if (remove_current_holder(handle.id)) {
            RCLCPP_DEBUG(m_logger, "Access released for cf %u", static_cast<unsigned>(handle.id));
        }
        m_access_requests.erase(handle.id);
    }

    void cancel(const AccessHandle & handle) override
    {
        const std::lock_guard<std::mutex> lock(m_holder_mutex);
        RCLCPP_INFO(m_logger, "Cancel requested for cf %d", handle.id);
        if (remove_current_holder(handle.id)) {
            RCLCPP_INFO(
                m_logger, "Access released for cf %u due to cancel",
                static_cast<unsigned>(handle.id));
        }
        m_access_requests.erase(handle.id);
    }

    bool get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position) override
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

    std::string get_pad_name(uint8_t id) const
    {
        return "pad_" + std::to_string(id);
    }
private: 
    struct CurrentHolder
    {
        uint8_t id;
        rclcpp::Time start_time;
    };

    bool is_current_holder(uint8_t id) const
    {
        return std::any_of(
            m_current_holders.begin(), m_current_holders.end(),
            [id](const CurrentHolder & holder) { return holder.id == id; });
    }

    bool remove_current_holder(uint8_t id)
    {
        const auto old_size = m_current_holders.size();
        std::erase_if(
            m_current_holders,
            [id](const CurrentHolder & holder) { return holder.id == id; });
        return m_current_holders.size() != old_size;
    }

    /// Converts resource-manager state to policy geometry and evaluates admission.
    /// Returns false if any required world-frame position cannot be resolved.
    bool can_grant_access(const AccessRequest & request)
    {
        AccessGeometry2D requesting_geometry;
        if (!get_access_geometry(request.id, requesting_geometry)) {
            return false;
        }

        std::vector<AccessGeometry2D> holder_geometries;
        holder_geometries.reserve(m_current_holders.size());
        for (const auto & holder : m_current_holders) {
            AccessGeometry2D holder_geometry;
            if (!get_access_geometry(holder.id, holder_geometry)) {
                return false;
            }
            holder_geometries.push_back(std::move(holder_geometry));
        }

        std::vector<AccessGeometry2D> all_access_geometries;
        all_access_geometries.reserve(m_access_requests.size());
        for (const auto & [id, access_request] : m_access_requests) {
            (void)access_request;
            AccessGeometry2D access_geometry;
            if (!get_access_geometry(id, access_geometry)) {
                return false;
            }
            all_access_geometries.push_back(std::move(access_geometry));
        }

        
        return m_access_policy->can_grant(
            requesting_geometry, holder_geometries, all_access_geometries);
    }

    /// Builds the ROS-independent 2D geometry for one stored access request.
    /// Both the live CF pose and its associated position are transformed to world.
    bool get_access_geometry(uint8_t id, AccessGeometry2D & geometry)
    {
        const auto request_it = m_access_requests.find(id);
        if (request_it == m_access_requests.end()) {
            return false;
        }

        geometry_msgs::msg::PoseStamped associated_position;
        if (!get_associated_position(id, associated_position) ||
            !m_megapad_tf->get_world_position_2d(
                request_it->second.current_pose, geometry.crazyflie_position) ||
            !m_megapad_tf->get_world_position_2d(
                associated_position, geometry.associated_position))
        {
            RCLCPP_DEBUG(
                m_logger, "Could not resolve 2D access geometry for cf %u",
                static_cast<unsigned>(id));
            return false;
        }
        return true;
    }

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
                const auto holder_it = std::find_if(
                    m_current_holders.begin(), m_current_holders.end(),
                    [ordered_id](const CurrentHolder & holder) {
                        return holder.id == ordered_id;
                    });
                if (holder_it != m_current_holders.end()) {
                    rclcpp::Duration time_held = now - holder_it->start_time;
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
    std::vector<CurrentHolder> m_current_holders;

    rclcpp::Duration p_max_hold_time = rclcpp::Duration::from_seconds(0.0);

    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> m_clock_interface;
    std::shared_ptr<MegaPadTF> m_megapad_tf;
    std::unique_ptr<MegaPadAccessPolicy> m_access_policy;

    std::unordered_map<uint8_t, AccessRequest> m_access_requests;
};

}  // namespace megapad
