#include "padflies_cpp/commander.hpp"

#include "padflies_cpp/command_land.hpp"
#include "padflies_cpp/command_takeoff.hpp"


PadflieCommander::PadflieCommander(
    const std::string & prefix,
    const std::string & cf_prefix,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeWaitablesInterface> node_waitables_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface)
: PadflieCommanderBase(prefix, cf_prefix, node_base_interface, node_param_interface, node_clock_interface, node_logging_interface)
, ICommandContext()
, m_node_base_interface(node_base_interface)
, m_node_timers_interface(node_timers_interface)
, m_node_clock_interface(node_clock_interface)
, m_pad_execute_server(std::make_shared<PadExecuteServer>(
    prefix,
    node_base_interface,
    node_clock_interface,
    node_logging_interface,
    node_waitables_interface,
    node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)))
, m_pad_client_factory(std::make_shared<PadClientFactory>(
    prefix,
    m_padflie_tf,
    node_base_interface,
    node_graph_interface,
    node_logging_interface,
    node_waitables_interface,
    node_services_interface,
    node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive),
    m_logger))
, m_pad_control(std::make_shared<PadControl>(
    prefix,
    node_base_interface,
    node_graph_interface,
    node_services_interface,
    node_waitables_interface,
    node_logging_interface))
, m_clock(node_clock_interface->get_clock())
{
    m_command_queue_timer = rclcpp::create_timer(
        node_base_interface,
        node_timers_interface,
        node_clock_interface->get_clock(),
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
        command->start();  
        m_state = command->get_working_state();
    }

    if (command->is_finished()) {
        m_state = command->get_target_state();
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
    return true; //return m_state == CommanderState::WAITING_FOR_TAKEOFF_RIGHTS;
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
    //m_pad_control->create_connection("megapad");
}

void PadflieCommander::m_on_commander_activated() 
{
    m_routine_factory = std::make_shared<RoutineFactory>(
        m_hardware_actor, 
        m_padflie_tf,
        m_pad_execute_server,
        m_pad_client_factory,
        m_node_base_interface,
        m_node_timers_interface,
        m_node_clock_interface, 
        m_logger);
    

    m_state = m_hw_state_controller.is_charged() ? CommanderState::CHARGED : CommanderState::CHARGING;
}

void 
PadflieCommander::m_deactivate_commander(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    bool force) 
{ 
    m_pad_control->destroy_connection(node);
    m_routine_factory.reset();
}

void PadflieCommander::m_on_commander_deactivated() 
{
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
        m_routine_factory,
        service_handle,
        request_id,
        req
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
        m_routine_factory,
        service_handle,
        request_id,
        req
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
        target.pose = Eigen::Affine3d::Identity();
        Eigen::Translation3d translation(
            msg->target.pose.position.x,
            msg->target.pose.position.y,
            msg->target.pose.position.z
        );
        Eigen::Quaterniond rotation(
            msg->target.pose.orientation.w,
            msg->target.pose.orientation.x,
            msg->target.pose.orientation.y,
            msg->target.pose.orientation.z
        );

        target.pose = translation * rotation;
        target.frame_id = msg->target.header.frame_id;
        target.use_yaw = msg->use_yaw;
        target.collision_avoidance = msg->collision_avoidance;
        m_hardware_actor->set_pose_target(target);
    }
}




