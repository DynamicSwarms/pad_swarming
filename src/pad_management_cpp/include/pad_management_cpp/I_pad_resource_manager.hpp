#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "class_loader/class_loader_core.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
namespace pad_management_cpp
{
    struct NodeInterfacesBundle
    {
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> base_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> topics_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> services_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> parameters_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> timers_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> logging_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> graph_interface;
        std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> waitables_interface;
    };


    struct AccessHandle
    {
        uint8_t id;
    };
    struct AccessRequest
    {
        uint8_t id;
        uint8_t action; // PadRightControlGoal::Action
        rclcpp::Duration max_wait_time{rclcpp::Duration::from_seconds(0.0)};
        rclcpp::Duration usage_time{rclcpp::Duration::from_seconds(0.0)};
        double battery_percentage{100.0};
        geometry_msgs::msg::PoseStamped current_pose;

        rclcpp::Time request_time{rclcpp::Time(0, 0, RCL_ROS_TIME)};
    };

    struct AccessResponse
    {
        enum class Result
        {
            ACCEPTED,
            REJECTED,
            PENDING
        } result;
        std::string message;
        rclcpp::Duration wait_time{rclcpp::Duration::from_seconds(0.0)}; 
    };

    struct ExecuteUpdate
    {
        uint8_t status; // PadExecuteFeedback::Status
        geometry_msgs::msg::PoseStamped current_pose;
        double battery_percentage;
    };

    struct ExecuteResult
    {
        uint8_t result; // PadExecuteResult::Result
    };
    


    struct AvailabilityStatus
    {
        bool available;
        enum class ChargingSpeed
        {
            NONE,
            SLOW,
            FAST
        } charging_speed;
        rclcpp::Duration wait_time{rclcpp::Duration::from_seconds(0.0)};
    };


class IPadResourceManager
{
public:

    virtual ~IPadResourceManager() = default;

    virtual AccessHandle submit_access_request(const AccessRequest & request) = 0;
    virtual AccessResponse query_request_status(const AccessHandle & handle) = 0;
    virtual void notify_update(const AccessHandle & handle, const ExecuteUpdate & update) = 0;
    virtual void notify_finished(const AccessHandle & handle, const ExecuteResult & result) = 0;
    virtual void cancel(const AccessHandle & handle) = 0;


    virtual std::vector<std::string> get_pad_tf_names() = 0;
    virtual bool get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position) = 0;

    void update_availability(AvailabilityStatus status)
    {
        m_last_status = status;
        if (m_on_change_callback) {
            m_on_change_callback(status);
        }
    }

    void set_on_change_callback(std::function<void(AvailabilityStatus)> callback)
    {
        m_on_change_callback = callback;
        callback(m_last_status);
    }

private: 
    std::function<void(AvailabilityStatus)> m_on_change_callback;
    AvailabilityStatus m_last_status = AvailabilityStatus
        {.available = false, 
         .charging_speed = AvailabilityStatus::ChargingSpeed::NONE, 
         .wait_time = rclcpp::Duration::from_seconds(1000.0)};
};

} // namespace pad_management_cpp


namespace class_loader
{



template<>
struct InterfaceTraits<pad_management_cpp::IPadResourceManager>
{
    using constructor_parameters =
        ConstructorParameters<pad_management_cpp::NodeInterfacesBundle>;
};

}
