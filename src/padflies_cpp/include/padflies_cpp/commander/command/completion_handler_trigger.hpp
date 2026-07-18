#pragma once

#include "padflies_cpp/commander/command/command.hpp"
#include "rclcpp/rclcpp.hpp"

class TriggerCompletionHandler : public ICompletionHandler
{
public:
    TriggerCompletionHandler(
      const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
      const std::shared_ptr<rmw_request_id_t> request_id,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request)
    : m_service_handle(service_handle)
    , m_request_id(request_id)
    , m_request(request)
    {};

    void on_succeeded() override
    {
        if (m_service_handle && m_request_id && m_request) {
            auto response = std_srvs::srv::Trigger::Response();
            response.success = true;
            response.message = "Command succeeded";
            m_service_handle->send_response(*m_request_id, response);
        }
    }

    void on_failed() override
    {
        if (m_service_handle && m_request_id && m_request) {
            auto response = std_srvs::srv::Trigger::Response();
            response.success = false;
            response.message = "Command failed";
            m_service_handle->send_response(*m_request_id, response);
        }
    }

    void on_aborted() override
    {
        if (m_service_handle && m_request_id && m_request) {
            auto response = std_srvs::srv::Trigger::Response();
            response.success = false;
            response.message = "Command aborted";
            m_service_handle->send_response(*m_request_id, response);
        }
    }

private:
    const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> m_service_handle;
    const std::shared_ptr<rmw_request_id_t> m_request_id;
    const std::shared_ptr<std_srvs::srv::Trigger::Request> m_request;
};