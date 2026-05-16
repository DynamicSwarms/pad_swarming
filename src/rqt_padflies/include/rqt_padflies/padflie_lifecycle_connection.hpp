#pragma once

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include <optional>
#include <future>

class PadflieLifecycleConnection
{
public:
    using LifecycleStateCallback = std::function<void(lifecycle_msgs::msg::State)>;

    PadflieLifecycleConnection(
        std::string prefix,
        LifecycleStateCallback lifecycle_state_callback,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        rclcpp::CallbackGroup::SharedPtr callback_group);   

    ~PadflieLifecycleConnection();

    void activate_padflie_with_callback(std::function<void(bool)> callback);
    void deactivate_padflie_with_callback(std::function<void(bool)> callback);

    bool padflie_is_available() {
        return m_get_state_client->service_is_ready();
    };
    
    void poll_current_lifecycle_state();

private: 
    void transition_padflie_with_callback(uint8_t id, const std::string &label, std::function<void(bool)> callback);

private: 
    void m_transition_event_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg);

private: 
    LifecycleStateCallback m_lifecycle_state_callback;
    std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> m_transition_event_sub;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> m_change_state_client;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> m_get_state_client;
};
