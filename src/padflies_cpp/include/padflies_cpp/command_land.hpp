#pragma once


#include "rclcpp/rclcpp.hpp"
#include "padflies_cpp/completion_handler_trigger.hpp"
#include "padflies_cpp/routine_factory.hpp"

class LandCommand : public Command
{
public:
    LandCommand(std::shared_ptr<RoutineFactory> routine_factory) 
    : Command(routine_factory->create_land_routine()) {};
    
    LandCommand(
      std::shared_ptr<RoutineFactory> routine_factory,
      const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> land_service_handle, 
      const std::shared_ptr<rmw_request_id_t> request_id,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request)
    : Command(routine_factory->create_land_routine(), std::make_shared<TriggerCompletionHandler>(land_service_handle, request_id, request))
    {};

    bool 
    preconditions_are_met(const ICommandContext& context) const override
    {
        bool can_land = context.can_land();
        bool is_healthy = context.is_healthy();
        return can_land && is_healthy;
    }

    CommanderState get_target_state() const override 
    {
        if (true)    
           return CommanderState::CHARGING;
    }

    CommanderState get_working_state() const override
    {
        return CommanderState::LANDING;
    }
};