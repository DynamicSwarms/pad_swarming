#pragma once

#include "visualization_msgs/msg/marker_array.hpp"
#include "rclcpp/rclcpp.hpp"

class SmartPadVisualization
{
public:
    SmartPadVisualization(
        uint8_t id,
        std::string pad_name,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        rclcpp::Logger parent_logger)
    : m_id(id)
    , m_pad_name(pad_name)
    , m_logger(parent_logger.get_child("SmartPadVisualization"))
    {
        auto pub_options = rclcpp::PublisherOptions();

        m_marker_publisher = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
            node_topics_interface,
            "smart_pad/visualization/markers",
            rclcpp::QoS(1).transient_local(),
            pub_options
        );

        publish_marker(); // Publish initial marker with default state
        RCLCPP_INFO(m_logger, "SmartPadVisualization initialized for pad %s", m_pad_name.c_str());
    }

    ~SmartPadVisualization() {
        publish_marker(true); 
        RCLCPP_INFO(m_logger, "SmartPadVisualization for pad %s is shutting down", m_pad_name.c_str());
    };

    void publish_marker(bool shutdown = false) const
    {
        visualization_msgs::msg::MarkerArray marker_array_msg;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = m_pad_name;
        marker.ns = m_pad_name;
        marker.id = m_id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.color = state_to_color(m_state);
        marker.color.a = 0.5; // Semi-transparent
        marker.scale.x = 0.2; 
        marker.scale.y = 0.2; 
        marker.scale.z = 0.04; 

        if (shutdown)
        {
            marker.action = visualization_msgs::msg::Marker::DELETE;
        }

        marker_array_msg.markers.push_back(marker);
        m_marker_publisher->publish(marker_array_msg);
        RCLCPP_INFO(m_logger, "Published marker for pad %s with state %u", m_pad_name.c_str(), m_state);
    }

    void set_state(uint8_t state)
    {
        RCLCPP_INFO(m_logger, "Setting state to %u", state);
        bool state_changed = (state != m_state);
        m_state = state;
        if (state_changed) {
            RCLCPP_INFO(m_logger, "State changed to %u, updating visualization", m_state);
            publish_marker();
        }
    }

private: 
    std_msgs::msg::ColorRGBA state_to_color(uint8_t state) const {
        auto color = std_msgs::msg::ColorRGBA();
        switch (state) {
            case 0: return make_color(0.0, 1.0, 0.0, 0.5); // Green for available
            case 1: return make_color(1.0, 1.0, 0.0, 0.5); // Yellow for locked
            case 2: return make_color(1.0, 0.0, 0.0, 0.5); // Red for occupied
            default: return make_color(1.0, 1.0, 1.0, 0.5); // White for unknown
        }
    }

    std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a) const {
        std_msgs::msg::ColorRGBA color;
        color.r = r;
        color.g = g;
        color.b = b;
        color.a = a;
        return color;
    }

private: 
    uint8_t m_state = 0;

private:
    uint8_t m_id;
    std::string m_pad_name;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> m_marker_publisher;
    rclcpp::Logger m_logger;
};