#pragma once

#include "rclcpp/rclcpp.hpp"
#include "smart_pad/smart_pad_tf.hpp"
#include "smart_pad/smart_pad_neighbors.hpp"
#include "smart_pad_interfaces/srv/lock.hpp"
#include <Eigen/Dense>

class SmartPadResource
{
public:
    SmartPadResource(
        std::string pad_name,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
        rclcpp::Logger parent_logger)
    : m_pad_name(pad_name)
    , m_logger(parent_logger.get_child("SmartPadResource"))
    {
        m_smart_pad_tf = std::make_shared<SmartPadTF>(
            pad_name,
            node_base_interface,
            node_topics_interface,
            node_clock_interface,
            m_logger
        );

        m_smart_pad_neighbors = std::make_unique<SmartPadNeighbors>(
            pad_name,
            m_smart_pad_tf,
            node_param_interface,
            m_logger
        );

        m_timer = rclcpp::create_timer(
            node_base_interface,
            node_timers_interface,
            node_clock_interface->get_clock(),
            std::chrono::milliseconds(1000), 
            std::bind(&SmartPadResource::timer_callback, this)
        );

        m_lock_service_callback_group = node_base_interface->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
        m_lock_service = rclcpp::create_service<smart_pad_interfaces::srv::Lock>(
            node_base_interface,
            node_services_interface,
            "~/lock",
            std::bind(&SmartPadResource::lock_service_callback, this, std::placeholders::_1, std::placeholders::_2),
            rclcpp::ServicesQoS(), 
            m_lock_service_callback_group
        );


        RCLCPP_INFO(m_logger, "SmartPadResource has been initialized.");
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
    std::string m_pad_name;
    rclcpp::Logger m_logger;
    std::shared_ptr<SmartPadTF> m_smart_pad_tf;
    std::unique_ptr<SmartPadNeighbors> m_smart_pad_neighbors;

    std::shared_ptr<rclcpp::TimerBase> m_timer;

    std::shared_ptr<rclcpp::CallbackGroup> m_lock_service_callback_group;
    std::shared_ptr<rclcpp::Service<smart_pad_interfaces::srv::Lock>> m_lock_service;
};