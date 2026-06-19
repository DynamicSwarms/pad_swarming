#pragma once
#include "rclcpp/rclcpp.hpp"
#include <mutex>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "class_loader/class_loader_core.hpp"
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
}

class IPadResourceManager
{
public:

    virtual ~IPadResourceManager() = default;

    bool try_lock(uint8_t id, uint8_t action)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_try_lock(id, action);
    };

    void release(uint8_t id, uint8_t result)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_release(id, result);
    };

    bool get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position)
    {
        return m_get_associated_position(id, position);
    }

    virtual std::vector<std::string> get_pad_tf_names() = 0;


    void update_availability(bool available)
    {
        if (m_on_change_callback) {
            m_on_change_callback(available);
        }
    }

    void set_on_change_callback(std::function<void(bool)> callback)
    {
        m_on_change_callback = callback;
    }

private: 
    virtual bool m_try_lock(uint8_t id, uint8_t action) = 0;

    // result is a uint8_t respresenting a result of pad_execute action
    virtual void m_release(uint8_t id, uint8_t result) = 0;

    virtual bool m_get_associated_position(uint8_t id, geometry_msgs::msg::PoseStamped & position) = 0;

    std::function<void(bool)> m_on_change_callback;
    std::mutex m_mutex;
};

namespace class_loader
{



template<>
struct InterfaceTraits<IPadResourceManager>
{
    using constructor_parameters =
        ConstructorParameters<pad_management_cpp::NodeInterfacesBundle>;
};

}