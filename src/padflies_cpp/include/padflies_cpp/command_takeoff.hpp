#pragma once


#include "rclcpp/rclcpp.hpp"
#include "padflies_cpp/command_trigger.hpp"
#include "padflies_cpp/routine_factory.hpp"

class TakeoffCommand : public TriggerCommand
{
public:
    TakeoffCommand(
      std::shared_ptr<RoutineFactory> routine_factory,
      const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> takeoff_service_handle, 
      const std::shared_ptr<rmw_request_id_t> request_id,
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request      
    ) :
        TriggerCommand(
            routine_factory->create_routine("TakeoffBehavior"),
            takeoff_service_handle,
            request_id,
            request
        )
    {

    };

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