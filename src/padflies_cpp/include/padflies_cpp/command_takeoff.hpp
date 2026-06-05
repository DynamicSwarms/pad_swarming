#pragma once


#include "rclcpp/rclcpp.hpp"
#include "padflies_cpp/completion_handler_trigger.hpp"
#include "padflies_cpp/routine_factory.hpp"

class TakeoffCommand : public Command
{
public:
    TakeoffCommand(std::shared_ptr<RoutineFactory> routine_factory) : Command(routine_factory->create_takeoff_routine()) {};

    TakeoffCommand(
        std::shared_ptr<RoutineFactory> routine_factory,
        std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
        std::shared_ptr<rmw_request_id_t> request_id,
        std::shared_ptr<std_srvs::srv::Trigger::Request> request)
        : Command(routine_factory->create_takeoff_routine(), std::make_shared<TriggerCompletionHandler>(service_handle, request_id, request))
    {};

    bool preconditions_are_met(const ICommandContext& context) const override
    {
        return context.is_healthy();
    }

    CommanderState get_target_state() const override 
    {
      if (true)    
           return CommanderState::FLYING;
    }

    CommanderState get_working_state() const override
    {
        return CommanderState::TAKEOFF;
    }
};