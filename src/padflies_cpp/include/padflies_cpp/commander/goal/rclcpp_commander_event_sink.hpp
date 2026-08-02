#pragma once

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "padflies_cpp/commander/goal/commander_event.hpp"

namespace padflies_cpp::commander
{

class RclcppCommanderEventSink final : public ICommanderEventSink
{
public:
  explicit RclcppCommanderEventSink(rclcpp::Logger logger)
  : m_logger(logger.get_child("FlightGoals")) {}

  void emit(const CommanderEvent & event) override
  {
    const auto goal = goal_name(event.goal_kind);

    switch (event.type) {
      case CommanderEventType::GOAL_REQUESTED:
        RCLCPP_INFO(
          m_logger, "Goal %llu requested: %s",
          static_cast<unsigned long long>(event.goal_id), goal);
        break;
      case CommanderEventType::GOAL_STARTED:
        RCLCPP_INFO(
          m_logger, "Goal %llu started: %s",
          static_cast<unsigned long long>(event.goal_id), goal);
        break;
      case CommanderEventType::INTERRUPTION_REQUESTED:
        RCLCPP_DEBUG(
          m_logger, "Interruption requested for goal %llu",
          static_cast<unsigned long long>(event.goal_id));
        break;
      case CommanderEventType::INTERRUPTION_ACCEPTED:
        RCLCPP_DEBUG(
          m_logger, "Interruption accepted for goal %llu",
          static_cast<unsigned long long>(event.goal_id));
        break;
      case CommanderEventType::INTERRUPTION_REJECTED:
        RCLCPP_DEBUG(
          m_logger, "Interruption not currently allowed for goal %llu: %s",
          static_cast<unsigned long long>(event.goal_id), event.detail.c_str());
        break;
      case CommanderEventType::GOAL_SUCCEEDED:
        RCLCPP_INFO(
          m_logger, "Goal %llu succeeded: %s",
          static_cast<unsigned long long>(event.goal_id), goal);
        break;
      case CommanderEventType::GOAL_FAILED:
        RCLCPP_WARN(
          m_logger, "Goal %llu failed: %s",
          static_cast<unsigned long long>(event.goal_id), event.detail.c_str());
        break;
      case CommanderEventType::GOAL_INTERRUPTED:
        RCLCPP_INFO(
          m_logger, "Goal %llu interrupted: %s",
          static_cast<unsigned long long>(event.goal_id), event.detail.c_str());
        break;
    }
  }

private:
  static const char * goal_name(FlightGoalKind kind)
  {
    switch (kind) {
      case FlightGoalKind::DEPLOY:
        return "deploy";
      case FlightGoalKind::DEPLOY_TO:
        return "deploy_to";
      case FlightGoalKind::RETURN:
        return "return";
      case FlightGoalKind::RETURN_TO:
        return "return_to";
    }
    return "unknown";
  }

  rclcpp::Logger m_logger;
};

}  // namespace padflies_cpp::commander
