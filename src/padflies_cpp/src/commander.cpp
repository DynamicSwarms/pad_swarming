#include "padflies_cpp/commander.hpp"

#define WORLD "world"

PadflieCommander::PadflieCommander(
    const std::string & prefix,
    const std::string & cf_prefix,
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_iface)
: m_prefix(prefix)
, m_cf_prefix(cf_prefix)
, m_charge_controller(param_iface)
, m_padflie_tf(cf_prefix.substr(1), WORLD)
, m_pad_control()
, m_pad_id(param_iface->declare_parameter("pad_id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
{
    m_padflie_tf.set_pad("pad_" + std::to_string(m_pad_id));    

    m_charge_controller.set_on_charged_callback(
        std::bind(&PadflieCommander::m_on_charged_callback, this));

}

void 
PadflieCommander::m_on_charged_callback()
{
    if (m_state == CommanderState::CHARGING)
        m_state = CommanderState::CHARGED;
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

    m_pad_control.on_activate(m_prefix, node);
    m_charge_controller.on_configure(m_cf_prefix, node);
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
    if (!m_charge_controller.is_charged()) {
        RCLCPP_ERROR(node->get_logger(), "Crazyflie is not charged!");
        return false;
    }

    
    m_pad_control.on_activate(m_prefix, node);
    m_padflie_actor = std::make_unique<PadflieActor>(node, m_cf_prefix, &m_padflie_tf);
    m_create_subscriptions(node);

    RCLCPP_INFO(node->get_logger(), "Padflie Commander activated for %s", m_cf_prefix.c_str());
    m_state = m_charge_controller.is_charged() ? CommanderState::CHARGED : CommanderState::CHARGING;
    return true; // Indicate successful activation
}

bool 
PadflieCommander::on_deactivate(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    if (m_state == CommanderState::UNCONFIGURED || m_state == CommanderState::CONFIGURED) {
        RCLCPP_ERROR(node->get_logger(), "Invalid state for deactivation: %d", static_cast<int>(m_state));
        return false;
    }
     

    m_remove_subscriptions(node); // First block all incomming commands
    m_deactivating = true;

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

    if (m_state != CommanderState::READY_TO_DEACTIVATE) {
        RCLCPP_ERROR(node->get_logger(), "PadflieCommander waiting for READY_TO_DEACTIVATE state!");
    }
    while (m_state != CommanderState::READY_TO_DEACTIVATE) rclcpp::sleep_for(std::chrono::milliseconds(10));
       
    m_padflie_actor.reset(); // Reset the actor to clean up resources
    m_pad_control.on_deactivate(node);

    RCLCPP_INFO(node->get_logger(), "Padflie Commander deactivated for %s", m_cf_prefix.c_str());
    m_deactivating = false;
    m_state = CommanderState::CONFIGURED;
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
    m_send_target_sub.reset();
}

void 
PadflieCommander::m_handle_info_timer()
{
    switch (m_state) {
        case CommanderState::CHARGED:
            {
            auto msg = std_msgs::msg::String();
            msg.data = m_prefix;
            m_availability_pub->publish(msg);
            }
            break;
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
            if (m_charge_controller.is_critical()) info_msg.battery = padflies_interfaces::msg::PadflieInfo::BATTERY_STATE_CRITICAL;
            else if (m_charge_controller.is_empty()) info_msg.battery = padflies_interfaces::msg::PadflieInfo::BATTERY_STATE_LOW;
            else info_msg.battery = padflies_interfaces::msg::PadflieInfo::BATTERY_STATE_OK;
            
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
        Eigen::Vector3d position;
        Eigen::Vector3d target_position;

        if (m_padflie_tf.get_cf_position(position)
            && m_pad_control.get_pad_circle_target(0.1, position, target_position))
        {
            geometry_msgs::msg::PoseStamped target_pose;
            target_pose.header.frame_id = WORLD;
            target_pose.pose.position.x = target_position.x();
            target_pose.pose.position.y = target_position.y();
            target_pose.pose.position.z = target_position.z();
            target_pose.header.frame_id = m_pad_control.get_frame_name();
            m_padflie_actor->set_target(target_pose, false);
                    
        }
    }        
}

void 
PadflieCommander::m_acquire_pad_right_callback(bool success)
{
    switch (m_state) {
        case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS:
            if (m_deactivating) {
                m_state = CommanderState::READY_TO_DEACTIVATE;
            } else if  (success) {
                m_state = CommanderState::TAKEOFF;
                m_padflie_actor->takeoff_routine(); // Blocking call, might take some time so in meantime m_deactivating might be set to true
                if (m_deactivating || m_state == CommanderState::LANDING) 
                {
                    if (m_state == CommanderState::LANDING) RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Weird transtion from TAKEOFF to LANDING");
                    m_padflie_actor->land_routine();
                    if (m_deactivating) m_state = CommanderState::READY_TO_DEACTIVATE;
                    else m_state = CommanderState::CHARGING; // After landing, we are in charging state
                }
                else m_state = CommanderState::FLYING;
            } else {
                m_state = CommanderState::CHARGING; 
            } 
            break;          
        case CommanderState::WAITING_FOR_LAND_RIGHTS:
            if (success)
            {
                m_state = CommanderState::LANDING;
                m_landing_target_timer->cancel(); 
                m_padflie_actor->land_routine();
                if (m_deactivating) m_state = CommanderState::READY_TO_DEACTIVATE;
                else m_state = CommanderState::CHARGING; // After landing, we are in charging state
            } 
            else 
            {
                if (m_deactivating) m_state = CommanderState::READY_TO_DEACTIVATE;
                else m_state = CommanderState::FLYING; // If we cannot acquire rights, we stay in flying state
            }          
            break;
        default:
            break;
    }    

    
    m_pad_control.release_right_async(
        [this](bool released) {
            if (released) {
                RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Released pad right successfully.");
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("PadflieCommander"), "Failed to release pad right.");
            }
        }
    );
    RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Right acquired, command executed, released, ready to deactivate.");
}

void 
PadflieCommander::m_trigger_landing()
{
    m_landing_target_timer->reset(); // Start sending landing targets

    m_pad_control.acquire_right_async(
        5.0, // Timeout for acquiring rights
        std::bind(&PadflieCommander::m_acquire_pad_right_callback, this, std::placeholders::_1));     
}

void 
PadflieCommander::m_trigger_takeoff()
{
    m_pad_control.acquire_right_async(
        5.0, // Timeout for acquiring rights
        std::bind(&PadflieCommander::m_acquire_pad_right_callback, this, std::placeholders::_1));
}


void 
PadflieCommander::m_handle_takeoff_command(
    const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg; 
    RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Takeoff command received for %s", m_cf_prefix.c_str());
    if (m_deactivating) return; // Reject any command while deactivating
    switch (m_state) {
        case CommanderState::CHARGED:
            m_state = CommanderState::WAITING_FOR_TAKEOFF_RIGHTS;
            m_trigger_takeoff();            
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("PadflieCommander"), "Cannot take off in current state: %d", static_cast<int>(m_state));
            break;
    }
}

void 
PadflieCommander::m_handle_land_command(
    const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Land command received for %s", m_cf_prefix.c_str());
    if (m_deactivating) return; // Reject any command while deactivating

    switch (m_state) {
        case CommanderState::FLYING:
            m_state = CommanderState::WAITING_FOR_LAND_RIGHTS;
            m_trigger_landing();
            break;
        case CommanderState::WAITING_FOR_TAKEOFF_RIGHTS: // Cancel takeoff command
            m_state = CommanderState::CHARGING;            
            break;
        case CommanderState::TAKEOFF:
            m_state = CommanderState::LANDING;
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("PadflieCommander"), "Cannot land in current state: %d", static_cast<int>(m_state));
            break;
    }
    
}

void 
PadflieCommander::m_handle_send_target_command(
    const padflies_interfaces::msg::SendTarget::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Send target command received for %s: %s", m_cf_prefix.c_str(), msg->info.c_str());
    if (m_deactivating) return; // Reject any command while deactivating
    if (m_padflie_actor && m_state == CommanderState::FLYING) {
        m_padflie_actor->set_target(msg->target, msg->use_yaw);
    }  
}