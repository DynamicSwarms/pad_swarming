#pragma once

#include "padflies_cpp/command.hpp"

#include "padflies_cpp/routine_factory.hpp"
#include "rclcpp/rclcpp.hpp"

class TriggerCommand : public Command
{
public:
    TriggerCommand(
      std::shared_ptr<Routine> routine,
      const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle, 
      const std::shared_ptr<rmw_request_id_t> request_id,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request)
    : Command(routine)
    , m_service_handle(service_handle)
    , m_request_id(request_id)
    , m_request(request)
    {}; 

    ~TriggerCommand() override  = default;

    void abort() override
    {
      if (m_service_handle && m_request_id && m_request) {
          auto response = std_srvs::srv::Trigger::Response();
          response.success = false;
          response.message = "Command aborted";
          m_service_handle->send_response(*m_request_id, response);
      }
    }

    void failed() override
    {
        if (m_service_handle && m_request_id && m_request) {
            auto response = std_srvs::srv::Trigger::Response();
            response.success = false;
            response.message = "Command failed";
            m_service_handle->send_response(*m_request_id, response);
        }
    }


    void succeeded() override {
        if (m_service_handle && m_request_id && m_request) {
            auto response = std_srvs::srv::Trigger::Response();
            response.success = true;
            response.message = "Command succeeded";
            m_service_handle->send_response(*m_request_id, response);
        }
    }

private: 
    const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> m_service_handle;
    const std::shared_ptr<rmw_request_id_t> m_request_id;
    const std::shared_ptr<std_srvs::srv::Trigger::Request> m_request;
};