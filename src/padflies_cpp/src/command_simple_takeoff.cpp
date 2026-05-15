#include "padflies_cpp/command_simple_takeoff.hpp"

SimpleTakeoffCommand::SimpleTakeoffCommand(
  std::shared_ptr<RoutineFactory> routine_factory, 
  const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> takeoff_service_handle, 
  const std::shared_ptr<rmw_request_id_t> request_id,
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request
)
    : TriggerCommand(
        routine_factory->create_routine("LandSimple"),
        takeoff_service_handle,
        request_id,
        request
      )
{


}