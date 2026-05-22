#include "padflies_cpp/hardware_actor.hpp"

static std::unordered_map<std::string, rclcpp::CallbackGroup::SharedPtr> m_callback_groups;
// https://github.com/ros2/rclcpp/pull/2683/commits/86d831375e8a7acdc55272866e04f4c214002414
// As soon as we switch to jazzy or newer we can make this a member variable, currently it would segfault on deconstruction

HardwareActor::HardwareActor(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeGraphInterface> node_graph_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeServicesInterface> node_services_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,    
    const std::string & cf_prefix, 
    std::shared_ptr<PadflieTF> padflie_tf
)
: m_state(ActorState::DEACTIVATED)
, m_mode(ActorMode::NONE)
, m_dt(0.1) // Default time step
, m_target_pose()
, m_fixed_yaw(false)
, m_yaw_controller(m_dt, 0.5) // Default max rotational velocity of 0.5 rad/s
, m_position_controller(m_dt, 5.0, 2.5, { 3.5, 4.0, 4.500, -7.5, -4.0, 0.0 }) // Default clipping box
, m_collision_avoidance_client(
    std::stoi(cf_prefix.substr(3)), 
    node_base_interface, 
    node_graph_interface, 
    node_services_interface,
    node_logging_interface->get_logger()) // Extract ID from cf_prefix (/cfID)
, m_hl_commander(
    node_base_interface,
    node_graph_interface,
    node_services_interface,
    node_logging_interface,
    cf_prefix)
, m_ll_commander(
    node_base_interface,
    node_topics_interface,
    node_graph_interface,
    node_services_interface,
    node_logging_interface,
    cf_prefix)
, m_padflie_tf(padflie_tf)
, m_logger(node_logging_interface->get_logger())
{
    if (m_callback_groups.find(cf_prefix) == m_callback_groups.end())
        m_callback_groups[cf_prefix] = node_base_interface->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

    m_send_target_timer = rclcpp::create_timer(
        node_base_interface,
        node_timers_interface, 
        node_clock_interface->get_clock(),
        std::chrono::milliseconds((long int)(m_dt * 1000)),
        std::bind(&HardwareActor::m_ll_command_timer_callback, this),
        m_callback_groups[cf_prefix]
    );
}

HardwareActor::~HardwareActor()
{
    
    RCLCPP_INFO(m_logger, "HardwareActor destructor called.");
}

std::string
HardwareActor::get_current_target_frame() const
{

    if (m_mode == ActorMode::POSITION_CONTROL) 
        return m_target_pose.frame_id;
    else if (m_mode == ActorMode::VELOCITY_CONTROL)
        return m_target_velocity.frame_id;
    else
        return "world";
}

bool 
HardwareActor::set_pose_target(const PoseTarget& target_pose) {
    if (m_state == ActorState::ERROR_STATE)
        return false;

    m_target_pose = target_pose;
    m_mode = ActorMode::POSITION_CONTROL;

    m_transition_to_low_level_commander();
    return true;
}

bool 
HardwareActor::set_velocity_target(
    const EigenVelocityStamped & velocity, 
    bool use_angular)
{
    if (m_state == ActorState::ERROR_STATE)
        return false;

    m_target_velocity = velocity;
    m_use_angular_velocity = use_angular;
    m_mode = ActorMode::VELOCITY_CONTROL;

    m_transition_to_low_level_commander();
    return true;
}

void 
HardwareActor::m_transition_to_low_level_commander()
{
    if (m_state == ActorState::HIGH_LEVEL_COMMANDER)
    {
        m_position_controller.initialize_target_history(m_target_pose.pose.translation());
        // What when velocity controller?
    }
    m_state = ActorState::LOW_LEVEL_COMMANDER;
}

void 
HardwareActor::m_transition_to_high_level_commander()
{
    if (m_state == ActorState::LOW_LEVEL_COMMANDER) 
    {
        m_ll_commander.notify_setpoints_stop(50);
    } 
    m_state = ActorState::HIGH_LEVEL_COMMANDER;
}

bool 
HardwareActor::go_to(
    const Eigen::Affine3d & target_pose,
    double duration,
    bool relative)
{
    if (m_state == ActorState::ERROR_STATE)
        return false;
        
    m_transition_to_high_level_commander();

    const double yaw_deg =
        std::atan2(target_pose.rotation()(1, 0), target_pose.rotation()(0, 0)) * 180.0 / M_PI;

    m_hl_commander.go_to(
        target_pose.translation(), 
        yaw_deg,                   
        duration,                   
        relative);
    return true;
}

bool
HardwareActor::land(
    double height, 
    double yaw, 
    double duration)
{
    if (m_state == ActorState::ERROR_STATE)
        return false;
    m_transition_to_high_level_commander();

    m_hl_commander.land(height, duration, yaw);
    return true;
}

bool
HardwareActor::takeoff(
    double height,
    double yaw,
    double duration)
{
    if (m_state == ActorState::ERROR_STATE)
        return false;
    m_transition_to_high_level_commander();
    
    m_hl_commander.takeoff(height, duration, yaw);
    return true;
}

void
HardwareActor::m_ll_command_timer_callback()
{
    if (m_state == ActorState::LOW_LEVEL_COMMANDER)
    {
        Eigen::Vector3d position;
        if (!m_padflie_tf->get_cf_position(position))
        {
            this->fail_safe("Failed to get current position for sending target.");
            // There have been rare cases wehere this fails. Maybe check twice, thrice etc?
            return;
        }

        if (m_mode == ActorMode::VELOCITY_CONTROL || m_mode == ActorMode::NONE) 
        {
            RCLCPP_INFO(m_logger, "Velocity Mode not supported yet.");
        }

        geometry_msgs::msg::PoseStamped set_target_pose;
        bool use_yaw;
        bool collision_avoidance;

        unpack_pose_target(m_target_pose, set_target_pose, use_yaw, collision_avoidance);
        Eigen::Vector3d target_position;
        double target_yaw;
        if (!m_padflie_tf->pose_stamped_to_world_position_and_yaw(set_target_pose, target_position, target_yaw))
        {
            // Target is not ok. Take last valid target. (Hover)
            // Or take cf position (which is also a hover)
            if (m_last_target_valid)
            {
                target_position = m_last_valid_target_position;
                target_yaw = m_last_valid_target_yaw;
            }
            else
            {
                target_position = position; // Use current position as target
                target_yaw = m_fixed_yaw_target; // Use current yaw
            }
            RCLCPP_INFO(m_logger, "Target pose not valid, using last valid target: (%f, %f, %f), yaw: %f",
                        target_position.x(), target_position.y(), target_position.z(), target_yaw);
        }
        if (!use_yaw) target_yaw = m_fixed_yaw_target;


        bool collision = false;
        if (collision_avoidance)
        {
            m_collision_avoidance_client.get_collision_avoidance_target(position, target_position, collision);
        }

        m_position_controller.safe_command_position(position, target_position, collision);
        double safe_yaw = m_yaw_controller.safe_cmd_yaw(m_current_yaw, target_yaw);
        
        // This is for race conditions and should be removed if possible.
        if (m_state == ActorState::LOW_LEVEL_COMMANDER)
        {
            m_ll_commander.cmd_position(target_position, safe_yaw);
        }
        
        m_current_yaw = safe_yaw; 
    }
}


void 
HardwareActor::fail_safe(std::string reason)
{
    m_state = ActorState::ERROR_STATE;
    m_hl_commander.land(
        -0.5,       // target height
        4.0,        // duration in seconds
        0.0);       // yaw
    RCLCPP_ERROR(m_logger, "Fail-safe triggered! Landing in place. %s", reason.c_str());
}