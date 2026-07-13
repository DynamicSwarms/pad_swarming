#pragma once

#include "rclcpp/rclcpp.hpp"
#include "smart_pad/smart_pad_neighbors.hpp"
#include "smart_pad_interfaces/srv/lock.hpp"

class NeighborsLock
{
    public:
    NeighborsLock(
        std::string pad_name,
        std::shared_ptr<SmartPadNeighbors> smart_pad_neighbors,
        std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface>     node_base_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface>    node_graph_interface,
        std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
        rclcpp::Logger parent_logger)
    : m_pad_name(pad_name)
    , m_smart_pad_neighbors(smart_pad_neighbors)
    , m_logger(parent_logger.get_child("NeighborsLock"))
    , m_node_base_interface(node_base_interface)
    , m_node_graph_interface(node_graph_interface)
    , m_node_services_interface(node_services_interface)
    , m_lock_service_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
    {}

    ~NeighborsLock() {
        RCLCPP_INFO(m_logger, "Releasing neighbors lock for pad %s", m_pad_name.c_str());
        for (const auto & lock_client : m_lock_clients) {
            auto request = std::make_shared<smart_pad_interfaces::srv::Lock::Request>();
            request->name = m_pad_name;
            request->locking = false;
            auto result_future = lock_client->async_send_request(request);
            if (result_future.wait_for(std::chrono::milliseconds(50)) == std::future_status::ready) {
                auto response = result_future.get();
                if (response->success) {
                    RCLCPP_WARN(m_logger, "Successfully released lock with neighbor");
                } else {
                    RCLCPP_ERROR(m_logger, "Failed to release lock with neighbor");
                }
            } else {
                RCLCPP_ERROR(m_logger, "Failed to release lock with neighbor, and did not receive response in time");
            }
        }
    }

bool try_lock() {
    std::vector<std::string> neighbors;
    if (!m_smart_pad_neighbors->get_neighbors(neighbors))
    {
        RCLCPP_ERROR(m_logger, "Failed to get neighbor list");
        return false;
    }
    for (const auto & neighbor : neighbors) 
    {
        std::shared_ptr<rclcpp::Client<smart_pad_interfaces::srv::Lock>> lock_client = 
           rclcpp::create_client<smart_pad_interfaces::srv::Lock>(
                m_node_base_interface,
                m_node_graph_interface,
                m_node_services_interface,
                neighbor + "/lock",
                rclcpp::ServicesQoS().keep_last(10),
                m_lock_service_callback_group
            );

        if (!lock_client->wait_for_service(std::chrono::milliseconds(50)))
        {
            RCLCPP_ERROR(m_logger, "Neighbor %s not available for locking", neighbor.c_str());
            return false;
        } 
        auto request = std::make_shared<smart_pad_interfaces::srv::Lock::Request>();
        request->name = m_pad_name;
        request->locking = true;
        auto result_future = lock_client->async_send_request(request);
        auto result = result_future.wait_for(std::chrono::milliseconds(50));
        if (result != std::future_status::ready)
        {
            RCLCPP_ERROR(m_logger, "Neighbor %s did not respond to lock request in time", neighbor.c_str());
            return false;
        } 
        auto response = result_future.get();
        if (!response->success) return false;
        
        RCLCPP_INFO(m_logger, "Successfully acquired lock with neighbor %s", neighbor.c_str());
        m_lock_clients.push_back(lock_client);
        RCLCPP_INFO(m_logger, "Successfully pushed lock client for neighbor %s", neighbor.c_str());
    }

    return true;
}


private:
    std::string m_pad_name;
    rclcpp::Logger m_logger;
    std::shared_ptr<SmartPadNeighbors> m_smart_pad_neighbors;
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> m_node_base_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> m_node_graph_interface;
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> m_node_services_interface;
    std::shared_ptr<rclcpp::CallbackGroup> m_lock_service_callback_group;

    std::vector<std::shared_ptr<rclcpp::Client<smart_pad_interfaces::srv::Lock>>> m_lock_clients;
};