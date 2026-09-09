#include "padflies_cpp/commander/actor/hardware_actor.hpp"
#include "padflies_cpp/commander/actor/collision_avoidance_client.hpp"

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
, m_position_controller(m_dt, 5.0, 2.5, { 3.5, 3.5, 4.500, -6.0, -2.5, 0.2 }) // Default clipping box
, m_velocity_controller(0.8, { 3.5, 3.5, 4.500, -6.0, -2.5, 0.2 })
, m_collision_avoidance_client(
    std::make_unique<CollisionAvoidanceClient>(
        std::stoi(cf_prefix.substr(2)), // Extract ID from cf_prefix (cfID)
        node_base_interface, 
        node_graph_interface, 
        node_services_interface,
        node_logging_interface->get_logger()))
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
    m_hardware_parameter_controller = std::make_shared<HardwareParameterController>(
        node_base_interface, 
        node_graph_interface, 
        node_services_interface, 
        cf_prefix, 
        m_logger);
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

    RCLCPP_INFO(m_logger, "HardwareActor initialized.");
}

HardwareActor::~HardwareActor()
{
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

    const bool switching_to_position_control =
        m_mode != ActorMode::POSITION_CONTROL;
    
    if (switching_to_position_control) {
    
        Eigen::Vector3d my_position = Eigen::Vector3d::Zero();
        if (!m_padflie_tf->get_cf_position(my_position))
        {
            RCLCPP_WARN(m_logger, "Failed to get current position for changing to position control.");
            my_position = m_target_pose.pose.translation(); 
        }
        m_position_controller.initialize_target_history(my_position);
    }
    m_target_pose = target_pose;
    m_mode = ActorMode::POSITION_CONTROL;

    m_transition_to_low_level_commander();
    return true;
}

bool 
HardwareActor::set_velocity_target(
    const VelocityTarget & velocity)
{
    if (m_state == ActorState::ERROR_STATE)
        return false;

    m_target_velocity = velocity;
    m_use_angular_velocity = velocity.use_angular;
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

    //const double yaw_deg =
    //    std::atan2(target_pose.rotation()(1, 0), target_pose.rotation()(0, 0)) * 180.0 / M_PI;

    double yaw = std::atan2(target_pose.rotation()(1, 0), target_pose.rotation()(0, 0));
    m_hl_commander.go_to(
        target_pose.translation(), 
        yaw,                   
        duration,                   
        relative);
    return true;
}

bool
HardwareActor::land(
    double height, 
    double yaw_rad, 
    double duration)
{
    if (m_state == ActorState::ERROR_STATE)
        return false;
    m_transition_to_high_level_commander();

    m_hl_commander.land(height, duration, yaw_rad);
    return true;
}

bool
HardwareActor::takeoff(
    double height,
    double yaw_rad,
    double duration)
{
    if (m_state == ActorState::ERROR_STATE)
        return false;
    m_transition_to_high_level_commander();
    
    m_hl_commander.takeoff(height, duration, yaw_rad);
    return true;
}

void
HardwareActor::reset_kalman_to(Eigen::Affine3d & pose)
{
    rcl_interfaces::msg::ParameterValue send_external_position_value;
    rcl_interfaces::msg::ParameterValue send_external_pose_value;
    if (m_hardware_parameter_controller->get_parameter("send_external_position", send_external_position_value) &&
        m_hardware_parameter_controller->get_parameter("send_external_pose", send_external_pose_value) &&
        send_external_position_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL &&
        send_external_pose_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL &&
        send_external_position_value.bool_value 
        && !send_external_pose_value.bool_value)
    {
        double pad_yaw = std::atan2(pose.rotation()(1, 0), pose.rotation()(0, 0));

                

        rclcpp::Parameter kalmanInitialX("kalman.initialX", pose.translation().x());
        rclcpp::Parameter kalmanInitialY("kalman.initialY", pose.translation().y());
        rclcpp::Parameter kalmanInitialZ("kalman.initialZ", pose.translation().z());
        rclcpp::Parameter kalmanInitialYaw("kalman.initialYaw", pad_yaw);
        std::vector<rcl_interfaces::msg::Parameter> params;
        params.push_back(kalmanInitialX.to_parameter_msg());
        params.push_back(kalmanInitialY.to_parameter_msg());
        params.push_back(kalmanInitialZ.to_parameter_msg());
        params.push_back(kalmanInitialYaw.to_parameter_msg());
        m_hardware_parameter_controller->set_parameters(params);
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
        std::vector<rcl_interfaces::msg::Parameter> kalman_params;
        rclcpp::Parameter kalmanReset("kalman.resetEstimation", 1);
        kalman_params.push_back(kalmanReset.to_parameter_msg());
        m_hardware_parameter_controller->set_parameters(kalman_params);
        m_padflie_tf->set_yaw(pad_yaw);
    } else {
        RCLCPP_DEBUG(m_logger, "Did not reset Kalman");  
    }
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

        if (m_mode == ActorMode::POSITION_CONTROL) 
        {
            m_do_cmd_position_update(position);
        } 
        else if (m_mode == ActorMode::VELOCITY_CONTROL) 
        {
            m_do_cmd_velocity_update(position);
        } 
        else 
        {
            RCLCPP_WARN(m_logger, "Unknown mode in m_ll_command_timer_callback.");
        }       
    }
}

void HardwareActor::m_do_cmd_position_update(Eigen::Vector3d & position)
{
    geometry_msgs::msg::PoseStamped set_target_pose;
    bool use_yaw, collision_avoidance;

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

    if (!std::isfinite(target_yaw))
    {
        RCLCPP_WARN(m_logger, "Target yaw is not finite. The set_target_pose is: (%f, %f, %f), quaternion: (%f, %f, %f, %f) ,frame_id: %s", 
                    set_target_pose.pose.position.x, set_target_pose.pose.position.y, set_target_pose.pose.position.z,
                    set_target_pose.pose.orientation.x, set_target_pose.pose.orientation.y, set_target_pose.pose.orientation.z, set_target_pose.pose.orientation.w,
                    set_target_pose.header.frame_id.c_str());
        target_yaw = m_fixed_yaw_target; // Use current yaw
    }

    bool collision = false;
    if (collision_avoidance)
    {
        // TODO(architecture): Remove this compatibility mirror once both command modes
        // publish a shared collision-object state consumed by both avoidance algorithms.
        m_collision_avoidance_client->mirror_target_to_velocity_avoidance(
            position, target_position, collision);
        //m_collision_avoidance_client->get_collision_avoidance_target(position, target_position, collision);
    }

    m_position_controller.safe_command_position(position, target_position, collision);
    double current_yaw = m_fixed_yaw_target;
    m_padflie_tf->get_yaw(current_yaw);
    double safe_yaw = m_yaw_controller.safe_cmd_yaw(current_yaw, target_yaw);
    RCLCPP_DEBUG(m_logger, "Current yaw: %f, Target yaw: %f, Safe yaw: %f", current_yaw, target_yaw, safe_yaw);
    m_padflie_tf->set_yaw(safe_yaw);

    
    // This is for race conditions and should be removed if possible.
    if (m_state == ActorState::LOW_LEVEL_COMMANDER)
    {
        m_ll_commander.cmd_position(target_position, safe_yaw * 180.0 / M_PI);
    }
}

void HardwareActor::m_do_cmd_velocity_update(Eigen::Vector3d & position)
{
    Eigen::Matrix<double, 6, 1> velocity_world;

    Eigen::Vector3d safe_velocity = Eigen::Vector3d::Zero();
    if (m_padflie_tf->velocity_transform(m_target_velocity.velocity, m_target_velocity.frame_id, "world", velocity_world))
    {
           safe_velocity = velocity_world.head<3>(); 
    } else {
        RCLCPP_WARN(m_logger, "Failed to transform velocity from frame %s to world frame. Sending zero velocity.", m_target_velocity.frame_id.c_str());
        velocity_world.setZero();
        safe_velocity.setZero();
    }

    bool collision = false;
    if (m_target_velocity.collision_avoidance)
    {
        m_collision_avoidance_client->get_collision_avoidance_velocity(
            position, safe_velocity, collision);
    }

    m_velocity_controller.safe_command_velocity(position, safe_velocity);

    if (collision)
    {
        RCLCPP_DEBUG(
            m_logger,
            "Velocity collision avoidance adjusted (%f, %f, %f) to (%f, %f, %f)",
            velocity_world.x(), velocity_world.y(), velocity_world.z(),
            safe_velocity.x(), safe_velocity.y(), safe_velocity.z());
    }

  


    if (m_state == ActorState::LOW_LEVEL_COMMANDER)
    {
        const double yaw_rate = m_use_angular_velocity ? velocity_world(5) : 0.0;
        m_ll_commander.cmd_velocity_world(safe_velocity, yaw_rate);
        m_padflie_tf->step_yaw(yaw_rate, m_dt);
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
