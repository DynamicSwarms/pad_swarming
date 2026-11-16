#pragma once

#include <memory>
#include <stdexcept>
#include <utility>

#include "rclcpp/service.hpp"
#include "rmw/types.h"
#include "std_srvs/srv/trigger.hpp"

#include "padflies_cpp/commander/goal/completion/goal_completion.hpp"

namespace padflies_cpp::commander
{

class TriggerGoalCompletion final : public IGoalCompletion
{
public:
  using Service = std_srvs::srv::Trigger;

  TriggerGoalCompletion(
    std::shared_ptr<rclcpp::Service<Service>> service,
    std::shared_ptr<rmw_request_id_t> request_id)
  : m_service(std::move(service)), m_request_id(std::move(request_id))
  {
    if (!m_service || !m_request_id) {
      throw std::invalid_argument("Trigger goal completion requires a service and request id");
    }
  }

  void complete(const GoalResult & result) override
  {
    if (m_completed) {
      throw std::logic_error("A trigger goal response was completed twice");
    }
    m_completed = true;

    Service::Response response;
    response.success = result.outcome == GoalOutcome::SUCCESS;
    response.message = result.message;
    if (result.outcome == GoalOutcome::INTERRUPTED && response.message.empty()) {
      response.message = "Flight goal interrupted";
    }
    m_service->send_response(*m_request_id, response);
  }

private:
  std::shared_ptr<rclcpp::Service<Service>> m_service;
  std::shared_ptr<rmw_request_id_t> m_request_id;
  bool m_completed{false};
};

}  // namespace padflies_cpp::commander
