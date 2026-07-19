#include "padflies_cpp/commander/commander.hpp"

#include "padflies_cpp/commander/command/command_land.hpp"
#include "padflies_cpp/commander/command/command_takeoff.hpp"
#include "padflies_cpp/commander/command/completion_handler_trigger.hpp"

PadflieCommander::PadflieCommander(
    const std::string & prefix,
    const std::string & cf_prefix,
    padflies_cpp::NodeInterfacesBundle node_interfaces_bundle
)
: PadflieCommanderBase(prefix, cf_prefix, node_interfaces_bundle)
, ICommandContext()
, m_node_base_interface(node_interfaces_bundle.base_interface)
, m_node_timers_interface(node_interfaces_bundle.timers_interface)
, m_node_clock_interface(node_interfaces_bundle.clock_interface)
, m_site_selector(std::make_shared<SiteSelector>(
    node_interfaces_bundle,
    m_padflie_tf,
    m_logger))
, m_routine_factory(std::make_shared<RoutineFactory>(
    node_interfaces_bundle,
    m_logger))
, m_clock(node_interfaces_bundle.clock_interface->get_clock())
{
    m_command_queue_timer = rclcpp::create_timer(
        node_interfaces_bundle.base_interface,
        node_interfaces_bundle.timers_interface,
        node_interfaces_bundle.clock_interface->get_clock(),
        std::chrono::milliseconds(100), // 10 Hz
        std::bind(&PadflieCommander::m_command_queue_execute, this),
        m_callback_group
    );
}

void
PadflieCommander::m_command_queue_execute()
{
    std::lock_guard<std::mutex> lock(m_command_queue_mutex);

    if (m_command_queue.empty()) return;

    std::shared_ptr<Command> command = m_command_queue.front();
    if (!command->has_been_started()) 
    {
        if (!command->preconditions_are_met(*this)) {
            RCLCPP_WARN(m_logger, "Preconditions for command not met, skipping command.");
            command->abort(); // No state update here??
            m_command_queue.pop();
            return;
        }
        RCLCPP_INFO(m_logger, "Starting command with target state %d", static_cast<int>(command->get_target_state()));
        if (!command->start()) {
            RCLCPP_ERROR(m_logger, "Could not start command: no suitable site or behavior plugin available.");
            m_command_queue.pop();
            return;
        }
        m_command_start_time = m_clock->now(); 
        m_state = command->get_working_state();
    }

    command->update();

    if (command->is_finished()) {
        m_state = command->get_target_state();
        auto duration = m_clock->now() - m_command_start_time;
        RCLCPP_INFO(m_logger, "Command finished with target state %d in %f seconds", static_cast<int>(command->get_target_state()), duration.seconds());
        m_command_queue.pop();
    }
}

void 
PadflieCommander::m_command_queue_on_deactivate()
{
    std::lock_guard<std::mutex> lock(m_command_queue_mutex);
    if (m_command_queue.empty()) return;

    std::shared_ptr<Command> command = m_command_queue.front();
    if (command->is_running()) {
        command->halt();
    }
}


bool 
PadflieCommander::is_healthy() const  
{
    bool isnt_flying = m_state == CommanderState::FLYING && !m_hw_state_controller.is_flying();
    bool cant_flying = m_state == CommanderState::FLYING && !m_hw_state_controller.canfly();
    return !isnt_flying && !cant_flying; 
}

bool 
PadflieCommander::get_home_state() const 
{
    return m_state != CommanderState::FLYING;
}

bool 
PadflieCommander::can_takeoff() const 
{
    return m_state == CommanderState::CHARGED;
}

bool 
PadflieCommander::can_land() const 
{
    return m_state == CommanderState::FLYING;
}

bool 
PadflieCommander::is_flying() const 
{
    return m_state == CommanderState::FLYING;
}


void 
PadflieCommander::m_configure_commander(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) 
{
}

void PadflieCommander::m_on_commander_configured() 
{
    m_state = CommanderState::CONFIGURED;
}

void
PadflieCommander::m_activate_commander(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) 
{
}

void PadflieCommander::m_on_commander_activated() 
{
    m_routine_factory->set_padflie_shared_ptrs(m_hardware_actor, m_padflie_tf);
    m_state = m_hw_state_controller.is_charged() ? CommanderState::CHARGED : CommanderState::CHARGING;
}

void 
PadflieCommander::m_deactivate_commander(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    bool force) 
{    
    if (force) return;

    std::shared_ptr<Command> command = std::make_shared<LandCommand>(
        m_routine_factory, m_site_selector, m_logger);
    
    {
        std::lock_guard<std::mutex> lock(m_command_queue_mutex);
        m_command_queue.push(command);
    }
    RCLCPP_INFO(m_logger, "Deactivating commander, landing padflie %s", m_cf_prefix.c_str());
    command->wait_until_finished();

}

void PadflieCommander::m_on_commander_deactivated() 
{
    m_routine_factory->reset_padflie_shared_ptrs();
    m_state = CommanderState::CONFIGURED;
}

void 
PadflieCommander::m_on_charged_callback() 
{
    if (m_state == CommanderState::CHARGING)
        m_state = CommanderState::CHARGED;
}

void 
PadflieCommander::m_handle_takeoff_command(
    const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
    const std::shared_ptr<rmw_request_id_t> request_id,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req) 
{
    RCLCPP_INFO(m_logger, "Takeoff command received for %s", m_cf_prefix.c_str());
    std::shared_ptr<Command> command = std::make_shared<TakeoffCommand>(
        m_routine_factory, m_site_selector, m_logger,
        std::make_shared<TriggerCompletionHandler>(service_handle, request_id, req)
    );
    {
        std::lock_guard<std::mutex> lock(m_command_queue_mutex);
        m_command_queue.push(command);
    }
}

void 
PadflieCommander::m_handle_land_command(
    const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
    const std::shared_ptr<rmw_request_id_t> request_id,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req) 
{   
    RCLCPP_INFO(m_logger, "Land command received for %s", m_cf_prefix.c_str());
    std::shared_ptr<Command> command = std::make_shared<LandCommand>(
        m_routine_factory, m_site_selector, m_logger,
        std::make_shared<TriggerCompletionHandler>(service_handle, request_id, req)
    );
    {
        std::lock_guard<std::mutex> lock(m_command_queue_mutex);
        m_command_queue.push(command);
    }
}

void 
PadflieCommander::m_handle_send_target_command(
    const padflies_interfaces::msg::SendTarget::SharedPtr msg) 
{
    if (m_state == CommanderState::FLYING) {
        PoseTarget target;
        tf2::fromMsg(msg->target.pose, target.pose);
        target.frame_id = msg->target.header.frame_id;

        target.use_yaw = msg->use_yaw;
        target.collision_avoidance = msg->collision_avoidance;
        m_hardware_actor->set_pose_target(target);
    }
}
