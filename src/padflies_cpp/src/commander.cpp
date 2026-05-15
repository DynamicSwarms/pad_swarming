#include "padflies_cpp/commander.hpp"

#include "padflies_cpp/command_simple_takeoff.hpp"
#include "padflies_cpp/command_simple_land.hpp"


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
, m_pad_control(std::make_shared<PadControl>(
    prefix,
    node_base_interface,
    node_graph_interface,
    node_services_interface,
    node_waitables_interface,
    node_logging_interface)
)
, m_clock(node_clock_interface->get_clock())
, m_bt_factory()
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



bool 
PadflieCommander::is_healthy() const  {
    bool isnt_flying = m_state == CommanderState::FLYING && !m_hw_state_controller.is_flying();
    bool cant_flying = m_state == CommanderState::FLYING && !m_hw_state_controller.canfly();
    return m_commander_is_healthy && !isnt_flying && !cant_flying; 
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
    return;
    // if (m_state != CommanderState::UNCONFIGURED) throw CommanderException("PadflieCommander is already configured!");

    //  m_landing_target_timer = rclcpp::create_timer(
    //      node->get_node_base_interface(),
    //      node->get_node_timers_interface(),
    //      node->get_node_clock_interface()->get_clock(),
    //      std::chrono::milliseconds(100), // 10 Hz
    //      std::bind(&PadflieCommander::m_handle_landing_target_timer, this),
    //      m_callback_group
    //  );
    //  m_landing_target_timer->cancel(); 
}
void PadflieCommander::m_on_commander_configured() 
{
    m_state = CommanderState::CONFIGURED;
}

void
PadflieCommander::m_activate_commander(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node) 
{
    m_commander_is_healthy = true;
    m_pad_control->create_connection("megapad");
}

void PadflieCommander::m_on_commander_activated() 
{
    m_routine_factory = std::make_shared<RoutineFactory>(
        m_hardware_actor, 
        m_pad_control,
        m_node_base_interface,
        m_node_timers_interface,
        m_node_clock_interface, 
        m_logger);
    

    RCLCPP_INFO(m_logger, "Commander activated");

    m_state = m_hw_state_controller.is_charged() ? CommanderState::CHARGED : CommanderState::CHARGING;
}

void 
PadflieCommander::m_deactivate_commander(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    bool force) 
{ 
    //m_deactivating = true;
    //
    //// If force flag is true, we dont care about if we are in the air (crazyflie tumbled)
    //if (force) {
    //    switch (m_state) {
    //        case CommanderState::WAITING_FOR_LAND_RIGHTS:
    //        case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS:
    //            m_state = CommanderState::FORCE_DEACTIVATE_RIGHT_WAIT;
    //            break;
    //        case CommanderState::LANDING:
    //        case CommanderState::TAKEOFF:
    //            break; // The padright callback will set the state to READY_TO_DEACTIVATE
    //        case CommanderState::CHARGING:
    //        case CommanderState::CHARGED:
    //        case CommanderState::FLYING:
    //            m_state = CommanderState::READY_TO_DEACTIVATE;
    //            break;           
    //    }
    //} else {
    //    switch (m_state) {
    //        case CommanderState::CHARGING:
    //        case CommanderState::CHARGED:
    //            m_state = CommanderState::READY_TO_DEACTIVATE;
    //            break;
    //        case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS:
    //        case CommanderState::TAKEOFF:
    //            break;
    //        case CommanderState::FLYING: 
    //            m_state = CommanderState::WAITING_FOR_LAND_RIGHTS;
    //            m_trigger_landing();
    //            break;
    //        case CommanderState::WAITING_FOR_LAND_RIGHTS:
    //        case CommanderState::LANDING:
    //            break;
    //        case CommanderState::READY_TO_DEACTIVATE:
    //            // This is impossible as the state transition is called from nodes lifecycle transitio
    //            // which is not parallel code
    //            throw CommanderException("Impossible state transition in PadflieCommander!");
    //        default:
    //            throw CommanderException("Invalid state in PadflieCommander during deactivation: " + std::to_string(static_cast<int>(m_state)));
    //    }
    // }

    // if (m_state != CommanderState::READY_TO_DEACTIVATE)
    //     RCLCPP_DEBUG(node->get_logger(), "PadflieCommander waiting for READY_TO_DEACTIVATE state!");
    // while (m_state != CommanderState::READY_TO_DEACTIVATE) m_clock->sleep_for(std::chrono::milliseconds(10));


    m_pad_control->destroy_connection(node);
}

void PadflieCommander::m_on_commander_deactivated() 
{
    m_state = CommanderState::CONFIGURED;
    m_commander_is_healthy = true;
}

void 
PadflieCommander::m_on_charged_callback() 
{
    if (m_state == CommanderState::CHARGING)
        m_state = CommanderState::CHARGED;
}

void 
PadflieCommander::m_handle_landing_target_timer()
{
//     if (m_state == CommanderState::WAITING_FOR_LAND_RIGHTS && m_hardware_actor ) {       
//         geometry_msgs::msg::PoseStamped current_pose;
//         geometry_msgs::msg::PoseStamped target_pose;
//         
//         if (m_padflie_tf.get_cf_pose_stamped("world", current_pose) &&
//             m_pad_control->get_pad_circle_target(0.1, current_pose, target_pose))
//         {
//             // m_padflie_actor->set_target(target_pose, false);        
//         }
//     }        
}

void 
PadflieCommander::m_acquire_pad_right_callback(bool success)
{
//    RCLCPP_INFO(m_logger, success ? "Acquired pad right successfully." : "Failed to acquire pad right.");
//    CommanderState new_state = m_state;
//
//    switch (m_state) {
//        case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS:
//            if (m_deactivating) {
//                new_state = CommanderState::READY_TO_DEACTIVATE;
//            } else if  (success) {
//                m_state = CommanderState::TAKEOFF;
//                //bool takeoff_success = m_padflie_actor->takeoff_routine(); // Blocking call, might take some time so in meantime m_deactivating might be set to true
//                
//                bool takeoff_success = true; // TODO: Implement proper takeoff routine and get the result here
//                if (!takeoff_success || !m_hw_state_controller.is_flying())
//                {
//                    RCLCPP_ERROR(m_logger, "Failed to takeoff");
//                    m_commander_is_healthy = false;
//                }
//
//
//                if (m_deactivating || m_state == CommanderState::LANDING) 
//                {
//                    if (m_state == CommanderState::LANDING) RCLCPP_INFO(m_logger, "Weird transtion from TAKEOFF to LANDING");
//                    //m_padflie_actor->land_routine();
//                    if (m_deactivating) new_state = CommanderState::READY_TO_DEACTIVATE;
//                    else new_state = CommanderState::CHARGING; // After landing, we are in charging state
//                }
//                else new_state = CommanderState::FLYING;
//            } else {
//                new_state = CommanderState::CHARGING; 
//            } 
//            break;          
//        case CommanderState::WAITING_FOR_LAND_RIGHTS:
//            if (success)
//            {
//                m_state = CommanderState::LANDING;
//                m_landing_target_timer->cancel(); 
//                //m_padflie_actor->land_routine();
//                if (m_deactivating) new_state = CommanderState::READY_TO_DEACTIVATE;
//                else new_state = CommanderState::CHARGING; // After landing, we are in charging state
//            } 
//            else 
//            {
//                if (m_deactivating) new_state = CommanderState::READY_TO_DEACTIVATE;
//                else m_trigger_landing(); // Retry landing
//            }          
//            break;
//        case CommanderState::FORCE_DEACTIVATE_RIGHT_WAIT:
//            new_state = CommanderState::READY_TO_DEACTIVATE;
//            break;
//        default:
//            break;
//    }    
//
//    // Release rights and only THEN change the state, otherwise use after free issue might occur
//    m_pad_control->release_right_async(
//        [this, new_state](bool released)
//        {
//            (void)released; // We don't care about the result of releasing rights
//            m_state = new_state;
//        }
//    );
}

void 
PadflieCommander::m_handle_takeoff_command(
    const std::shared_ptr<rclcpp::Service<std_srvs::srv::Trigger>> service_handle,
    const std::shared_ptr<rmw_request_id_t> request_id,
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req) 
{
    RCLCPP_INFO(m_logger, "Takeoff command received for %s", m_cf_prefix.c_str());
    std::shared_ptr<Command> command = std::make_shared<SimpleTakeoffCommand>(
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
    std::shared_ptr<Command> command = std::make_shared<SimpleLandCommand>(
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
        EigenPoseStamped target;
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
        m_hardware_actor->set_pose_target(target, msg->use_yaw);
    }
}




//bool 
//PadflieCommander::m_process_takeoff_command() 
//{
//
//    return true;
//    if (m_routine) 
//    {
//        RCLCPP_WARN(m_logger, "Takeoff command received but a routine is already running.");
//        m_routine->halt(); // Stop the currently running routine before starting a new one
//        return false;
//    }
//    m_routine = m_routine_factory->create_routine("TakeoffSimple");
//    m_routine->set_on_finished_callback(std::bind(&PadflieCommander::m_on_takeoff_finished, this, std::placeholders::_1));
//    m_routine->start();
//
//    return true;
//
//    if (m_deactivating) return false; // Reject any command while deactivating
//    switch (m_state) {
//        case CommanderState::CHARGED:
//            m_state = CommanderState::WAITING_FOR_TAKEOFF_RIGHTS;
//            m_pad_control->acquire_right_async(
//                60.0, // Timeout for acquiring rights
//                std::bind(&PadflieCommander::m_acquire_pad_right_callback, this, std::placeholders::_1));         
//            break;
//        default:
//            RCLCPP_ERROR(m_logger, "Cannot take off in current state: %d", static_cast<int>(m_state));
//            break;
//    }
//    return true;    
//}
//
void 
PadflieCommander::m_trigger_landing()
{
//    m_landing_target_timer->reset(); // Start sending landing targets
//
//    m_pad_control->acquire_right_async(
//        180.0, // Timeout for acquiring rights
//        std::bind(&PadflieCommander::m_acquire_pad_right_callback, this, std::placeholders::_1)); 
}
//
//bool 
//PadflieCommander::m_process_land_command() 
//{
//    if (m_routine) 
//    {
//        RCLCPP_WARN(m_logger, "Land command received but a routine is already running.");
//        m_routine->halt(); // Stop the currently running routine before starting a new one
//        return false;
//    }
//
//    m_routine = m_routine_factory->create_routine("LandSimple");
//    m_routine->set_on_finished_callback(std::bind(&PadflieCommander::m_on_land_finished, this, std::placeholders::_1));
//    m_routine->start();
//    return true;
//    if (m_deactivating) return false; // Reject any command while deactivating
//
//    switch (m_state) {
//        case CommanderState::FLYING:
//            m_state = CommanderState::WAITING_FOR_LAND_RIGHTS;
//            m_trigger_landing();
//            break;
//        case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS: // Cancel takeoff command
//            m_state = CommanderState::CHARGING;            
//            break;
//        case CommanderState::TAKEOFF:
//            m_state = CommanderState::LANDING;
//            break;
//        default:
//            RCLCPP_ERROR(m_logger, "Cannot land in current state: %d", static_cast<int>(m_state));
//            break;
//    } 
//    return true;  
//}




