#include "padflies_cpp/actor.hpp"


PadflieActor::PadflieActor(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    const std::string & cf_prefix, 
    PadflieTF* padflie_tf
)
: m_state(ActorState::DEACTIVATED)
, m_dt(0.1) // Default time step
, m_target_pose()
, m_fixed_yaw(false)
, m_yaw_controller(m_dt, 0.5) // Default max rotational velocity of 0.5 rad/s
, m_position_controller(m_dt, 3.0, 1.5, { 3.5, 4.0, 0.000, -7.5, -2.0, 4.500 }) // Default clipping box
, m_hl_commander(node, cf_prefix)
, m_ll_commander(node, cf_prefix)
, m_padflie_tf(padflie_tf)
{
    // m_send_target_timer = node->create_wall_timer(
    //     std::chrono::milliseconds(100), // 100 ms interval
    //     std::bind(&PadflieActor::m_send_target_callback, this)
    // );
}

PadflieActor::~PadflieActor()
{
    //m_send_target_timer.reset(); // Reset the timer to stop it
    RCLCPP_INFO(rclcpp::get_logger("PadflieActor"), "PadflieActor destructor called for ");
}

void PadflieActor::set_target(
    const geometry_msgs::msg::PoseStamped & target_pose,
    bool use_yaw)
{
    m_target_pose = target_pose;
    m_fixed_yaw = !use_yaw;
}

void PadflieActor::get_target_pose(
    geometry_msgs::msg::PoseStamped & target_pose) const
{
    target_pose = m_target_pose;
}

bool PadflieActor::takeoff_routine(
    double takeoff_height)
{
    if (m_state == ActorState::ERROR_STATE)
        return false;
    
    m_state = ActorState::HIGH_LEVEL_COMMANDER;
    geometry_msgs::msg::PoseStamped target_pose;
    if (!m_padflie_tf->get_pad_pose_world(target_pose))
    {
        RCLCPP_ERROR(rclcpp::get_logger("PadflieActor"), "Failed to get pad position for takeoff.");
        return false;
    }

    target_pose.pose.position.z += takeoff_height;
    this->set_target(target_pose, false); // Set target so that after takeoff we will hover.

    m_hl_commander.go_to(
        Eigen::Vector3d(0,0,0.1), // relative position to current position
        0.0,                      // yaw
        1.0,                      // duration in seconds
        true);                    // relative movement

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
 
    m_hl_commander.go_to(
        Eigen::Vector3d(0,0,0.6), // relative position to current position
        0.0,                      // yaw
        4.0,                      // duration in seconds
        true);                    // relative movement

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Even though we specified more time for takeoff, this ensures a cleaner transition.
    m_state = ActorState::LOW_LEVEL_COMMANDER;
    return true; // Indicate successful takeoff
}

/** Routine for LAND.
 *  Lands the crazyflie safely into the pad.
 *
 *  Precondition: Crazyflie is in the air, not too far from the Pad (max. 0.75 Meters)
 *  After: Crazyflie is seated in the pad. Motors are off. State is HIGH Level Commander
 */     
bool PadflieActor::land_routine()
{
    if (m_state == ActorState::ERROR_STATE)
        return false;
    m_ll_commander.notify_setpoints_stop(200);

    /**
     * Phase2: 
     *  We are now only using HighLevelCommander
     *  Use GoTos to properly land. 
     *  We were 0.5 meters above the pad. 
     *  First Step: lower to 0.2 m above pad. This is because Phase1 probably overshoots
     *  Second Step: Fly straight down into the pad. For a good seating.
     *
     *  During this phase a moving pad is bad but we need the clean high level command routines for safe flight. 
     *
     *  Between each command the pad position gets querried again.
    */
    double pad_yaw; 
    Eigen::Vector3d pad_position;

    if (!m_padflie_tf->get_pad_position_and_yaw(pad_position, pad_yaw))
    {
        m_fail_safe("No pad position found for landing (Phase2a).");
        return false;
    }

    m_hl_commander.go_to(
        pad_position + Eigen::Vector3d(0, 0, 0.25),// global position above pad
        pad_yaw,                                   // yaw
        4.5);                                      // duration in seconds

    std::this_thread::sleep_for(std::chrono::milliseconds(4500));

    if (!m_padflie_tf->get_pad_position_and_yaw(pad_position, pad_yaw))
    {
        m_fail_safe("No pad position found for landing (Phase2b).");
        return false;
    }

    m_hl_commander.go_to(
        pad_position + Eigen::Vector3d(0, 0, -0.1), // global position in pad
        pad_yaw,                                    // yaw
        1.5);                                       // duration in seconds
    
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    /**
     * Phase3:
     * We are already in the Pad. 
     * We land to stop the motors and this will also wiggle us to properly seat in the pad.
    */
    if (!m_padflie_tf->get_pad_position_and_yaw(pad_position, pad_yaw))
    {
        m_fail_safe("No pad position found for landing (Phase3).");
        return false;
    }

    m_hl_commander.land(
        -0.5,       // target height
        2.5,        // duration in seconds
        pad_yaw);   // yaw
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true; // Indicate successful landing
}

double PadflieActor::get_yaw() const
{
    return m_current_yaw;
}

void PadflieActor::m_send_target_callback()
{
    if (m_state == ActorState::LOW_LEVEL_COMMANDER)
    {
        Eigen::Vector3d position;
        if (!m_padflie_tf->get_cf_position(position))
        {
            m_fail_safe("Failed to get current position for sending target.");
            return;
        }

        Eigen::Vector3d target_position;
        double target_yaw;
        if (!m_padflie_tf->pose_stamped_to_world_position_and_yaw(m_target_pose, target_position, target_yaw))
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
        }

        if (m_fixed_yaw) target_yaw = m_fixed_yaw_target;

        Eigen::Vector3d safe_target;
        m_position_controller.safe_command_position(position, target_position, safe_target);
        double safe_yaw = m_yaw_controller.safe_cmd_yaw(m_current_yaw, target_yaw);

        // This is for race conditions and should be removed if possible.
        if (m_state == ActorState::LOW_LEVEL_COMMANDER)
        {
            m_ll_commander.cmd_position(safe_target, safe_yaw);
        }
        
        m_current_yaw = safe_yaw; 
    }
}

void PadflieActor::m_fail_safe(std::string reason)
{
    m_state = ActorState::ERROR_STATE;
    RCLCPP_ERROR(rclcpp::get_logger("PadflieActor"), "Fail-safe triggered! Landing in place. %s", reason.c_str());
}