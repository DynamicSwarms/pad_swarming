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
{
    m_pad_id = param_iface->declare_parameter("pad_id", rclcpp::ParameterValue(0), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>();

    m_charge_controller.set_on_charged_callback(
        std::bind(&PadflieCommander::m_on_charged_callback, this));

    m_padflie_tf.set_pad("pad_" + std::to_string(m_pad_id));
}

void PadflieCommander::m_on_charged_callback()
{
    if (m_state == CommanderState::CHARGING)
        m_state = CommanderState::CHARGED;
}



bool PadflieCommander::on_configure(
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

    m_pad_control.configure(m_prefix, node);
    m_charge_controller.on_configure(m_cf_prefix, node);
    m_padflie_tf.on_configure(node);

    m_state = CommanderState::CHARGING;
    return true; // Indicate successful configuration
}

bool PadflieCommander::on_activate(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    if (m_state != CommanderState::CHARGED) {
        RCLCPP_ERROR(node->get_logger(), "PadflieCommander is not in CHARGED state!");
        return false;
    }
    
    m_pad_control.configure(m_prefix, node);


    m_padflie_actor = std::make_unique<PadflieActor>(node, m_cf_prefix, &m_padflie_tf);

    m_create_subscriptions(node);

    RCLCPP_INFO(node->get_logger(), "Padflie Commander activated for %s", m_cf_prefix.c_str());
    return true; // Indicate successful activation
}

bool PadflieCommander::on_deactivate(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    // Deactivate might be used for landing
    if (!(m_state == CommanderState::CHARGED || m_state == CommanderState::CHARGING)) {
        RCLCPP_ERROR(node->get_logger(), "PadflieCommander is not in CHARGED or CHARGING state!");
        return false;
    }

    m_pad_control.unconfigure(node);

    m_remove_subscriptions(node);
    m_padflie_actor.reset(); // Reset the actor to clean up resources

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

    m_send_target_sub = node->create_subscription<padflies_interfaces::msg::SendTarget>(
        m_prefix + "/send_target", 10,
        std::bind(&PadflieCommander::m_handle_send_target_command, this, std::placeholders::_1),
        sub_options);
}

void PadflieCommander::m_remove_subscriptions(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    m_takeoff_sub.reset();
    m_land_sub.reset();
    m_send_target_sub.reset();
}

void PadflieCommander::m_handle_landing_target_timer()
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

void PadflieCommander::m_acquire_pad_right_callback(bool success)
{
    if (success) {
        if (m_pending_command == PendingCommand::TAKEOFF) {
            m_state = CommanderState::TAKEOFF;
            m_padflie_actor->takeoff_routine();
            m_state = CommanderState::FLYING;
        } else if (m_pending_command == PendingCommand::LAND) {
            m_landing_target_timer->cancel();
            m_state = CommanderState::LANDING;
            m_padflie_actor->land_routine();
            m_state = CommanderState::CHARGING;
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

    } else {
        RCLCPP_ERROR(rclcpp::get_logger("PadflieCommander"), "Failed to acquire pad right. Canceling command: %s",
            m_pending_command == PendingCommand::TAKEOFF ? "Takeoff" : "Land");
    }

    
    m_pending_command = PendingCommand::NONE; // Reset pending command
}

void PadflieCommander::m_handle_takeoff_command(
    const std_msgs::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Takeoff command received for %s", m_cf_prefix.c_str());
    switch (m_state) {
        case CommanderState::CHARGED:
            if (m_pending_command != PendingCommand::NONE) {
                RCLCPP_WARN(rclcpp::get_logger("PadflieCommander"), "Pending command already exists, ignoring takeoff command. This should not be able to happen.");
                return;
            }

            m_pending_command = PendingCommand::TAKEOFF;
            m_state = CommanderState::WAITING_FOR_TAKEOFF_RIGHTS;

            m_pad_control.acquire_right_async(
                5.0, // Timeout for acquiring rights
                std::bind(&PadflieCommander::m_acquire_pad_right_callback, this, std::placeholders::_1));
            break;
        default:
            // WAITING_FOR_LANDING_RIGHTS might be interesting to consider for future
            RCLCPP_ERROR(rclcpp::get_logger("PadflieCommander"), "Cannot take off in current state: %d", static_cast<int>(m_state));
            break;
    }
}

void PadflieCommander::m_handle_land_command(
    const std_msgs::msg::Empty::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Land command received for %s", m_cf_prefix.c_str());
    switch (m_state) {
        case CommanderState::FLYING:
            if (m_pending_command != PendingCommand::NONE) {
                RCLCPP_WARN(rclcpp::get_logger("PadflieCommander"), "Pending command already exists, ignoring land command. This should not be able to happen.");
                return;
            }

            m_pending_command = PendingCommand::LAND;
            m_state = CommanderState::WAITING_FOR_LAND_RIGHTS;
            m_landing_target_timer->reset(); 

            m_pad_control.acquire_right_async(
                5.0, // Timeout for acquiring rights
                std::bind(&PadflieCommander::m_acquire_pad_right_callback, this, std::placeholders::_1));
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("PadflieCommander"), "Cannot land in current state: %d", static_cast<int>(m_state));
            break;
    }
    
}

void PadflieCommander::m_handle_send_target_command(
    const padflies_interfaces::msg::SendTarget::SharedPtr msg)
{
    RCLCPP_INFO(rclcpp::get_logger("PadflieCommander"), "Send target command received for %s: %s", m_cf_prefix.c_str(), msg->info.c_str());
    if (m_padflie_actor && m_state == CommanderState::FLYING) {
        m_padflie_actor->set_target(msg->target, msg->use_yaw);
    }
    
    
}