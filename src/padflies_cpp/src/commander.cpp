#include "padflies_cpp/commander.hpp"

#include "rcl_interfaces/msg/parameter_value.hpp"
#include "rcl_interfaces/msg/parameter.hpp"


#define WORLD "world"

PadflieCommander::PadflieCommander(
    const std::string & prefix,
    const std::string & cf_prefix,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_iface)
: m_prefix(prefix)
, m_cf_prefix(cf_prefix)
, m_hw_state_controller(param_iface)
, m_padflie_tf(cf_prefix.substr(1), WORLD)
, m_pad_control()
, m_secondary_pad_control()
, m_pad_id(param_iface->declare_parameter("pad_id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
, m_logger_name(prefix)
, m_param_iface(param_iface)
, m_hw_param_controller()
{
    m_pad_control_reset();

    m_hw_state_controller.set_on_charged_callback(
        std::bind(&PadflieCommander::m_on_charged_callback, this));

}

bool 
PadflieCommander::is_healthy() const {
    bool isnt_flying = m_state == CommanderState::FLYING && !m_hw_state_controller.is_flying();
    bool cant_flying = m_state == CommanderState::FLYING && !m_hw_state_controller.canfly();
    return m_commander_is_healthy && !isnt_flying && !cant_flying; 
}

void 
PadflieCommander::m_on_charged_callback()
{
    if (m_state == CommanderState::CHARGING)
        m_state = CommanderState::CHARGED;
}

void 
PadflieCommander::m_pad_control_reset()
{
    m_padflie_tf.set_pad("pad_" + std::to_string(m_pad_id));    
    m_current_pad_control = &m_pad_control;
}

bool 
PadflieCommander::on_configure(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    if (m_state != CommanderState::UNCONFIGURED) {
        RCLCPP_ERROR(node->get_logger(), "PadflieCommander is already configured!");
        return false;
    }

    if (!m_callback_group) {
        m_callback_group = node->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);
    }

    m_landing_target_timer = node->create_wall_timer(
        std::chrono::milliseconds(100), // 100 ms interval
        std::bind(&PadflieCommander::m_handle_landing_target_timer, this),
        m_callback_group
    );
    m_landing_target_timer->cancel(); 

    m_pad_control.set_node(node); 
    m_pad_control.activate("megapad", m_prefix);
    m_secondary_pad_control.set_node(node);
    m_current_pad_control = &m_pad_control;

    m_hw_param_controller.configure(node, m_cf_prefix);


    m_hw_state_controller.on_configure(m_cf_prefix, node);
    m_padflie_tf.on_configure(node);

    auto pub_options = rclcpp::PublisherOptions();
    pub_options.callback_group = m_callback_group;
    m_availability_pub = node->create_publisher<std_msgs::msg::String>(
        "availability", 10, pub_options);

    m_padflie_info_pub = node->create_publisher<padflies_interfaces::msg::PadflieInfo>(
        m_prefix + "/info", 10, pub_options);

    m_info_timer = node->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PadflieCommander::m_handle_info_timer, this),
            m_callback_group
    );

    m_state = CommanderState::CONFIGURED;
    return true; // Indicate successful configuration
}

bool PadflieCommander::on_activate(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    if (m_state != CommanderState::CONFIGURED) {
        RCLCPP_ERROR(node->get_logger(), "PadflieCommander is not in CONFIGURED state!");
        return false;
    }
    if (!m_hw_state_controller.is_charged()) {
        RCLCPP_ERROR(node->get_logger(), "Crazyflie is not charged!");
        return false;
    }
    if  (!m_hw_state_controller.canfly()) {
        RCLCPP_ERROR(node->get_logger(), "Crazyflie cannot fly!");
        return false;
    }

    m_commander_is_healthy = true;
    m_padflie_actor = std::make_unique<PadflieActor>(node, m_cf_prefix, &m_padflie_tf);
    m_create_subscriptions(node);

    RCLCPP_INFO(node->get_logger(), "Padflie Commander activated for %s", m_cf_prefix.c_str());
    m_state = m_hw_state_controller.is_charged() ? CommanderState::CHARGED : CommanderState::CHARGING;
    return true; // Indicate successful activation
}

bool 
PadflieCommander::on_deactivate(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    bool force)
{
    if (force) m_pad_control_reset();
    if (m_state == CommanderState::UNCONFIGURED || m_state == CommanderState::CONFIGURED) {
        RCLCPP_ERROR(node->get_logger(), "Invalid state for deactivation: %d", static_cast<int>(m_state));
        return false;
    }
     
    m_remove_subscriptions(node); // First block all incomming commands
    m_deactivating = true;

    // If force flag is true, we dont care about if we are in the air (crazyflie tumbled)
    if (force) {
        switch (m_state) {
            case CommanderState::WAITING_FOR_LAND_RIGHTS:
            case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS:
                m_state = CommanderState::FORCE_DEACTIVATE_RIGHT_WAIT;
                break;
            case CommanderState::LANDING:
            case CommanderState::TAKEOFF:
                break; // The padright callback will set the state to READY_TO_DEACTIVATE
            case CommanderState::CHARGING:
            case CommanderState::CHARGED:
            case CommanderState::FLYING:
                m_state = CommanderState::READY_TO_DEACTIVATE;
                break;           
        }
    } else {
        switch (m_state) {
            case CommanderState::CHARGING:
            case CommanderState::CHARGED:
                m_state = CommanderState::READY_TO_DEACTIVATE;
                break;
            case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS:
            case CommanderState::TAKEOFF:
                break;
            case CommanderState::FLYING: 
                m_state = CommanderState::WAITING_FOR_LAND_RIGHTS;
                m_trigger_landing();
                break;
            case CommanderState::WAITING_FOR_LAND_RIGHTS:
            case CommanderState::LANDING:
                break;
            case CommanderState::READY_TO_DEACTIVATE:
                // This is impossible as the state transition is called from nodes lifecycle transitio
                // which is not parallel code
                RCLCPP_ERROR(node->get_logger(), "Impossible state transition!");
                return false;
            default:
                RCLCPP_ERROR(node->get_logger(), "Padflie Commander tries to deactivate but is in an invalid state: %d", static_cast<int>(m_state));
                return false;
        }
    }

    if (m_state != CommanderState::READY_TO_DEACTIVATE)
        RCLCPP_INFO(node->get_logger(), "PadflieCommander waiting for READY_TO_DEACTIVATE state!");
    while (m_state != CommanderState::READY_TO_DEACTIVATE) rclcpp::sleep_for(std::chrono::milliseconds(10));
       
    m_padflie_actor.reset(); // Reset the actor to clean up resources

    m_deactivating = false;
    m_commander_is_healthy = true;
    m_hw_state_controller.reset_state();
    m_state = CommanderState::CONFIGURED;
    RCLCPP_INFO(node->get_logger(), "Padflie Commander deactivated for %s", m_cf_prefix.c_str());
    return true; // Indicate successful deactivation
}



void PadflieCommander::m_create_subscriptions(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = m_callback_group;

    m_takeoff_sub = node->create_subscription<std_msgs::msg::Empty>(
        m_prefix + "/pad_takeoff", 10,
        std::bind(&PadflieCommander::m_handle_takeoff_command, this, std::placeholders::_1),
        sub_options);

    m_land_sub = node->create_subscription<std_msgs::msg::Empty>(
        m_prefix + "/pad_land", 10,
        std::bind(&PadflieCommander::m_handle_land_command, this, std::placeholders::_1),
        sub_options);

    m_land_at_sub = node->create_subscription<std_msgs::msg::String>(
        m_prefix + "/pad_land_at", 10,
        std::bind(&PadflieCommander::m_handle_land_at_command, this, std::placeholders::_1),
        sub_options);

    m_send_target_sub = node->create_subscription<padflies_interfaces::msg::SendTarget>(
        m_prefix + "/send_target", 10,
        std::bind(&PadflieCommander::m_handle_send_target_command, this, std::placeholders::_1),
        sub_options);
}

void PadflieCommander::m_remove_subscriptions(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    (void)node;
    m_takeoff_sub.reset();
    m_land_sub.reset();
    m_land_at_sub.reset();
    m_send_target_sub.reset();
}

void 
PadflieCommander::m_handle_info_timer()
{
    switch (m_state) {
        case CommanderState::CONFIGURED:
            if (m_hw_state_controller.is_charged() && 
                m_hw_state_controller.canfly() &&
                !m_hw_state_controller.is_tumbled())
            {
                auto msg = std_msgs::msg::String();
                msg.data = m_prefix;
                m_availability_pub->publish(msg);
            }
            break;
        case CommanderState::CHARGING:
        case CommanderState::CHARGED:
        case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS:
        case CommanderState::TAKEOFF:
        case CommanderState::FLYING:
        case CommanderState::WAITING_FOR_LAND_RIGHTS:
        case CommanderState::LANDING:
            {
            padflies_interfaces::msg::PadflieInfo info_msg;
            info_msg.cf_prefix = m_cf_prefix;
            if (m_padflie_tf.get_cf_pose_stamped(m_padflie_actor->get_current_target_frame(), info_msg.pose)) 
                info_msg.pose_valid = true;
            Eigen::Vector3d position;
            if (m_padflie_tf.get_cf_position(position))
            {    
                info_msg.pose_world_valid = true;
                info_msg.pose_world.position.x = position.x();
                info_msg.pose_world.position.y = position.y();
                info_msg.pose_world.position.z = position.z();
            }
            info_msg.is_home = m_state == CommanderState::WAITING_FOR_TAKEOFF_RIGHTS;
            if (m_hw_state_controller.is_critical()) info_msg.battery = padflies_interfaces::msg::PadflieInfo::BATTERY_STATE_CRITICAL;
            else if (m_hw_state_controller.is_empty()) info_msg.battery = padflies_interfaces::msg::PadflieInfo::BATTERY_STATE_LOW;
            else info_msg.battery = padflies_interfaces::msg::PadflieInfo::BATTERY_STATE_OK;

            info_msg.padflie_state = 1; // STATE ISNT USED YET, but 0 throws an error
            
            m_padflie_info_pub->publish(info_msg);
            }
            break;
        default:
            // Do nothing for other states
            break;
    }
}

void 
PadflieCommander::m_handle_landing_target_timer()
{
    if (m_state == CommanderState::WAITING_FOR_LAND_RIGHTS && m_padflie_actor ) {       
        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::PoseStamped target_pose;
        
        if (m_padflie_tf.get_cf_pose_stamped("world", current_pose) &&
            m_current_pad_control->get_pad_circle_target(0.1, current_pose, target_pose))
        {
            m_padflie_actor->set_target(target_pose, false);        
        }
    }        
}

void 
PadflieCommander::m_acquire_pad_right_callback(bool success)
{
    RCLCPP_INFO(rclcpp::get_logger(m_logger_name), success ? "Acquired pad right successfully." : "Failed to acquire pad right.");
    CommanderState new_state = m_state;
    PadControl * current_pad_control = m_current_pad_control;

    switch (m_state) {
        case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS:
            if (m_deactivating) {
                new_state = CommanderState::READY_TO_DEACTIVATE;
            } else if  (success) {
                m_state = CommanderState::TAKEOFF;
                m_reset_yaw_if_needed();
                bool takeoff_success = m_padflie_actor->takeoff_routine(); // Blocking call, might take some time so in meantime m_deactivating might be set to true
                if (!takeoff_success || !m_hw_state_controller.is_flying())
                {
                    RCLCPP_ERROR(rclcpp::get_logger(m_logger_name), "Failed to takeoff");
                    m_commander_is_healthy = false;
                }


                if (m_deactivating || m_state == CommanderState::LANDING) 
                {
                    if (m_state == CommanderState::LANDING) RCLCPP_INFO(rclcpp::get_logger(m_logger_name), "Weird transtion from TAKEOFF to LANDING");
                    m_padflie_actor->land_routine();
                    if (m_deactivating) new_state = CommanderState::READY_TO_DEACTIVATE;
                    else new_state = CommanderState::CHARGING; // After landing, we are in charging state
                }
                else new_state = CommanderState::FLYING;
            } else {
                new_state = CommanderState::CHARGING; 
            } 
            break;          
        case CommanderState::WAITING_FOR_LAND_RIGHTS:
            if (success)
            {
                m_state = CommanderState::LANDING;
                m_landing_target_timer->cancel(); 
                m_padflie_actor->land_routine();
                if (m_deactivating) new_state = CommanderState::READY_TO_DEACTIVATE;
                else new_state = CommanderState::CHARGING; // After landing, we are in charging state
            } 
            else 
            {
                if (m_deactivating) new_state = CommanderState::READY_TO_DEACTIVATE;
                else {
                    m_pad_control_reset();
                    m_trigger_landing(); // Retry landing at original pad
                }
            }          
            break;
        case CommanderState::FORCE_DEACTIVATE_RIGHT_WAIT:
            new_state = CommanderState::READY_TO_DEACTIVATE;
            break;
        default:
            break;
    }    

    // Release rights and only THEN change the state, otherwise use after free issue might occur
    current_pad_control->release_right_async(
        [this, new_state](bool released)
        {
            (void)released; // We don't care about the result of releasing rights
            m_state = new_state;
        }
    );
}

void 
PadflieCommander::m_reset_yaw_if_needed()
{
    rcl_interfaces::msg::ParameterValue send_external_position_value;
    rcl_interfaces::msg::ParameterValue send_external_pose_value;
    if (m_hw_param_controller.get_parameter("send_external_position", send_external_position_value) &&
        m_hw_param_controller.get_parameter("send_external_pose", send_external_pose_value))
    {
        if (send_external_position_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL &&
            send_external_pose_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_BOOL)
        {

            if (send_external_position_value.bool_value && !send_external_pose_value.bool_value)
            {    
                Eigen::Vector3d pad_position; 
                double yaw; 
                if (m_padflie_tf.get_pad_position_and_yaw(pad_position, yaw)) {
                    rclcpp::Parameter kalmanInitialX("kalman.initialX", pad_position.x());
                    rclcpp::Parameter kalmanInitialY("kalman.initialY", pad_position.y());
                    rclcpp::Parameter kalmanInitialZ("kalman.initialZ", pad_position.z());
                    rclcpp::Parameter kalmanInitialYaw("kalman.initialYaw", yaw);
                    rclcpp::Parameter kalmanReset("kalman.resetEstimation", 1);
             
                    std::vector<rcl_interfaces::msg::Parameter> params;
                    params.push_back(kalmanInitialX.to_parameter_msg());
                    params.push_back(kalmanInitialY.to_parameter_msg());
                    params.push_back(kalmanInitialZ.to_parameter_msg());
                    params.push_back(kalmanInitialYaw.to_parameter_msg());
                    params.push_back(kalmanReset.to_parameter_msg());
                    m_hw_param_controller.set_parameters(params);
                    std::this_thread::sleep_for(std::chrono::milliseconds(200)); // Wait for the parameters to be set
                } else {
                    RCLCPP_ERROR(rclcpp::get_logger(m_logger_name), "Failed to get pad position and yaw cannot reset kalman");
                    return;
                }
            }    
        }
    }
}

void 
PadflieCommander::m_trigger_landing(const std::string & pad_name)
{
    bool can_transform = true;  
    if (pad_name == "" || !(can_transform = m_padflie_tf.can_transform(pad_name)))
    {
        m_pad_control_reset(); // set to homepad

        if (!can_transform) RCLCPP_INFO(rclcpp::get_logger(m_logger_name), "Pad %s is not in tf graph landing at megabase", pad_name.c_str());
    } else 
    {
        m_padflie_tf.set_pad(pad_name);    
        m_secondary_pad_control.deactivate();                 // Remove any existing secondary pad_control
        m_secondary_pad_control.activate(pad_name, m_prefix); // Activate the offsite pad_control for new pad
        m_current_pad_control = &m_secondary_pad_control;
    }


    m_landing_target_timer->reset(); // Start sending landing targets

    m_current_pad_control->acquire_right_async(
        180.0, // Timeout for acquiring rights
        std::bind(&PadflieCommander::m_acquire_pad_right_callback, this, std::placeholders::_1));     
}

void 
PadflieCommander::m_trigger_takeoff()
{
    m_current_pad_control->acquire_right_async(
        60.0, // Timeout for acquiring rights
        std::bind(&PadflieCommander::m_acquire_pad_right_callback, this, std::placeholders::_1));
}


void 
PadflieCommander::m_handle_takeoff_command(
    const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg; 
    RCLCPP_INFO(rclcpp::get_logger(m_logger_name), "Takeoff command received for %s", m_cf_prefix.c_str());
    if (m_deactivating) return; // Reject any command while deactivating
    switch (m_state) {
        case CommanderState::CHARGED:
            m_state = CommanderState::WAITING_FOR_TAKEOFF_RIGHTS;
            m_trigger_takeoff();            
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger(m_logger_name), "Cannot take off in current state: %d", static_cast<int>(m_state));
            break;
    }
}

void 
PadflieCommander::m_handle_land_command(
    const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(rclcpp::get_logger(m_logger_name), "Land command received for %s", m_cf_prefix.c_str());
    m_process_land_command();
}

void 
PadflieCommander::m_handle_land_at_command(
    const std_msgs::msg::String::SharedPtr msg
)
{
    std::string pad_name = msg->data;
    RCLCPP_INFO(rclcpp::get_logger(m_logger_name), "Land at command received for %s, at pad %s", m_cf_prefix.c_str(), pad_name.c_str());
    m_process_land_command(msg->data);
}

void 
PadflieCommander::m_process_land_command(const std::string & pad_name)
{
    if (m_deactivating) return; // Reject any command while deactivating
    switch (m_state) {
        case CommanderState::FLYING:
            m_state = CommanderState::WAITING_FOR_LAND_RIGHTS;
            m_trigger_landing(pad_name);
            break;
        case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS: // Cancel takeoff command
            m_state = CommanderState::CHARGING;            
            break;
        case CommanderState::TAKEOFF:
            m_state = CommanderState::LANDING;
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger(m_logger_name), "Cannot land in current state: %d", static_cast<int>(m_state));
            break;
    }
}



void 
PadflieCommander::m_handle_send_target_command(
    const padflies_interfaces::msg::SendTarget::SharedPtr msg)
{
    //RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Send target command received for %s: %d", m_cf_prefix.c_str(), static_cast<int>(m_state));
    if (m_deactivating) return; // Reject any command while deactivating
    if (!m_padflie_actor) return;

    // Accept the target only if we are flying or 
    // in the transition phase -> smoother takeoff
    switch (m_state) {
        case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS:
        case CommanderState::TAKEOFF:
        case CommanderState::FLYING:
            m_padflie_actor->set_target(msg->target, msg->use_yaw);
            break;
        default:
            break;
    } 
}