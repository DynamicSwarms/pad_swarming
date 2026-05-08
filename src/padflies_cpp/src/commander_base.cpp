#include "padflies_cpp/commander_base.hpp"

#define WORLD "world"

PadflieCommanderBase::PadflieCommanderBase(
    const std::string & prefix,
    const std::string & cf_prefix,
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> node_param_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface
)
: m_prefix(prefix)
, m_cf_prefix(cf_prefix)
, m_hw_state_controller(node_param_interface)
, m_padflie_tf(cf_prefix.substr(1), WORLD, node_clock_interface->get_clock(), node_logging_interface->get_logger())
, m_callback_group(node_base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
, m_initial_pad(node_param_interface->declare_parameter("initial_pad", rclcpp::ParameterValue(""), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<std::string>())
, m_param_callback_handle(node_param_interface->add_on_set_parameters_callback(std::bind(&PadflieCommanderBase::m_set_parameters_callback, this, std::placeholders::_1)))
, m_node_clock_interface(node_clock_interface)
, m_logger(node_logging_interface->get_logger())
{
    if (!m_initial_pad.empty()) {
        m_padflie_tf.set_pad(m_initial_pad);
    } else {
    // TODO
    }
  
    m_hw_state_controller.set_on_state_callback(
        std::bind(&PadflieCommanderBase::m_on_state_callback, this));
    m_hw_state_controller.set_on_charged_callback(
        std::bind(&PadflieCommanderBase::m_on_charged_callback, this));
}

PadflieCommanderBase::~PadflieCommanderBase() = default;

void 
PadflieCommanderBase::on_configure(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    if (m_base_state != CommanderBaseState::UNCONFIGURED) throw CommanderException("PadflieCommanderBase is already configured!");

    m_configure_commander(node);

    m_hw_state_controller.connect(m_cf_prefix, node);
    m_padflie_tf.start_listening(node);
  
    m_create_availability_interface(node);

    m_on_commander_configured();
    m_base_state = CommanderBaseState::CONFIGURED;

    RCLCPP_INFO(node->get_logger(), "Padflie Commander configured for %s", m_cf_prefix.c_str());
}

void 
PadflieCommanderBase::on_activate(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    if (m_base_state != CommanderBaseState::CONFIGURED) throw CommanderException("PadflieCommanderBase is not configured!");
    if (!m_hw_state_controller.is_charged()) throw CommanderException("Crazyflie is not charged!");
    if (!m_hw_state_controller.canfly()) throw CommanderException("Crazyflie cannot fly!");
    Eigen::Vector3d position;
    if (!m_padflie_tf.get_cf_position(position)) throw CommanderException("Crazyflie position is not available!");

    m_activate_commander(node);

    m_hardware_actor = std::make_shared<HardwareActor>(
        node->get_node_base_interface(),
        node->get_node_topics_interface(),
        node->get_node_graph_interface(),
        node->get_node_services_interface(),
        node->get_node_timers_interface(),
        node->get_node_clock_interface(),
        node->get_node_logging_interface(),
        m_cf_prefix,
        &m_padflie_tf);

    m_remove_availability_interface(node);
    m_create_control_interface(node);

    m_on_commander_activated();
    m_base_state = CommanderBaseState::ACTIVATED;
    RCLCPP_INFO(node->get_logger(), "Padflie Commander activated for %s", m_cf_prefix.c_str());
}

void 
PadflieCommanderBase::on_deactivate(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node,
    bool force)
{ 
    if (m_base_state != CommanderBaseState::ACTIVATED) throw CommanderException("PadflieCommanderBase is not activated, invalid transition!");

    m_remove_control_interface(node); // First block all incomming commands

    m_deactivate_commander(node, force);
    
    m_hardware_actor.reset(); // Reset the actor to clean up resources
    
    m_hw_state_controller.reset_state();
    m_base_state = CommanderBaseState::CONFIGURED;
    RCLCPP_INFO(node->get_logger(), "Padflie Commander deactivated for %s", m_cf_prefix.c_str());
}

void PadflieCommanderBase::m_on_state_callback()
{
    bool charged = m_hw_state_controller.is_charged();
    bool canfly = m_hw_state_controller.canfly();
    bool tumbled = m_hw_state_controller.is_tumbled();

    if (charged && canfly && !tumbled)
    {
        auto msg = std_msgs::msg::String();
        msg.data = m_prefix;
        if (m_availability_pub) m_availability_pub->publish(msg);
    }
}

void 
PadflieCommanderBase::m_handle_info_timer()
{
    padflies_interfaces::msg::PadflieInfo info_msg;
    info_msg.cf_prefix = m_cf_prefix;
    // if (m_padflie_tf.get_cf_pose_stamped(m_hardware_actor->get_current_target_frame(), info_msg.pose)) 
    //     info_msg.pose_valid = true;
    Eigen::Vector3d position;
    if (m_padflie_tf.get_cf_position(position))
    {    
        info_msg.pose_world_valid = true;
        info_msg.pose_world.position.x = position.x();
        info_msg.pose_world.position.y = position.y();
        info_msg.pose_world.position.z = position.z();
    }
    info_msg.is_home = get_home_state();
    if (m_hw_state_controller.is_critical()) info_msg.battery = padflies_interfaces::msg::PadflieInfo::BATTERY_STATE_CRITICAL;
    else if (m_hw_state_controller.is_empty()) info_msg.battery = padflies_interfaces::msg::PadflieInfo::BATTERY_STATE_LOW;
    else info_msg.battery = padflies_interfaces::msg::PadflieInfo::BATTERY_STATE_OK;

    info_msg.padflie_state = 1; // STATE ISNT USED YET, but 0 throws an error
    
    m_padflie_info_pub->publish(info_msg);
}

void PadflieCommanderBase::m_create_availability_interface(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    auto pub_options = rclcpp::PublisherOptions();
    pub_options.callback_group = m_callback_group;

    m_availability_pub = node->create_publisher<std_msgs::msg::String>(
        "availability", 10, pub_options);
}

void PadflieCommanderBase::m_remove_availability_interface(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    (void)node;
    m_availability_pub.reset();
}


void PadflieCommanderBase::m_create_control_interface(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface = node->get_node_base_interface();
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface = node->get_node_timers_interface();
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface = node->get_node_clock_interface();

    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = m_callback_group;

    auto pub_options = rclcpp::PublisherOptions();
    pub_options.callback_group = m_callback_group;

    m_takeoff_service = node->create_service<std_srvs::srv::Trigger>(
        m_prefix + "/takeoff", 
        std::bind(&PadflieCommanderBase::m_handle_takeoff_command, this, std::placeholders::_1, std::placeholders::_2));

    m_land_service = node->create_service<std_srvs::srv::Trigger>(
        m_prefix + "/land", 
        std::bind(&PadflieCommanderBase::m_handle_land_command, this, std::placeholders::_1, std::placeholders::_2));

    m_send_target_sub = node->create_subscription<padflies_interfaces::msg::SendTarget>(
        m_prefix + "/send_target", 10,
        std::bind(&PadflieCommanderBase::m_handle_send_target_command, this, std::placeholders::_1),
        sub_options);

    m_padflie_info_timer = rclcpp::create_timer(
        node_base_interface,
        node_timers_interface,
        node_clock_interface->get_clock(),
        std::chrono::milliseconds(100),
        std::bind(&PadflieCommanderBase::m_handle_info_timer, this),
        m_callback_group
    );

    m_padflie_info_pub = node->create_publisher<padflies_interfaces::msg::PadflieInfo>(
        m_prefix + "/info", 10, pub_options);
}

void PadflieCommanderBase::m_remove_control_interface(
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    (void)node;
    m_takeoff_service.reset();
    m_land_service.reset();
    m_send_target_sub.reset();

    m_padflie_info_timer->cancel();
    m_padflie_info_timer.reset();
    m_padflie_info_pub.reset();
}


void 
PadflieCommanderBase::m_handle_takeoff_command(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    RCLCPP_INFO(m_logger, "Takeoff command received for %s", m_cf_prefix.c_str());
    res->success = m_process_takeoff_command();
    res->message = "Takeoff command processed";
}

void 
PadflieCommanderBase::m_handle_land_command(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
    (void)req;
    RCLCPP_INFO(m_logger, "Land command received for %s", m_cf_prefix.c_str());
    
    res->success = m_process_land_command();
    res->message = "Land command processed";    
}


rcl_interfaces::msg::SetParametersResult 
PadflieCommanderBase::m_set_parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";
    for (const auto &param : parameters)
    {
        if (param.get_name() == "abc")
        {

        }
         
    }
    return result;
}
