#include "padflies_cpp/commander/commander_base.hpp"

#define WORLD "world"

PadflieCommanderBase::PadflieCommanderBase(
    const std::string & prefix,
    const std::string & cf_prefix,
    padflies_cpp::NodeInterfacesBundle node_interfaces_bundle
)
: m_prefix(prefix)
, m_cf_prefix(cf_prefix)
, m_node_interfaces(std::move(node_interfaces_bundle))
, m_hw_state_controller(m_node_interfaces.parameters_interface)
, m_padflie_tf(std::make_shared<PadflieTF>(cf_prefix, WORLD, m_node_interfaces.clock_interface->get_clock(), m_node_interfaces.logging_interface->get_logger()))
, m_callback_group(m_node_interfaces.base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive))
, m_param_callback_handle(m_node_interfaces.parameters_interface->add_on_set_parameters_callback(std::bind(&PadflieCommanderBase::m_set_parameters_callback, this, std::placeholders::_1)))
, m_node_clock_interface(m_node_interfaces.clock_interface)
, m_logger(m_node_interfaces.logging_interface->get_logger())
{ 
    m_hw_state_controller.set_on_state_callback(
        std::bind(&PadflieCommanderBase::m_on_state_callback, this));
    m_hw_state_controller.set_on_charged_callback(
        std::bind(&PadflieCommanderBase::m_on_charged_callback, this));
}

PadflieCommanderBase::~PadflieCommanderBase() = default;

void 
PadflieCommanderBase::on_configure()
{
    if (m_base_state != CommanderBaseState::UNCONFIGURED) throw CommanderException("PadflieCommanderBase is already configured!");

    m_configure_commander();

    m_hw_state_controller.connect(m_cf_prefix, m_node_interfaces);
    m_padflie_tf->start_listening(
        m_node_interfaces.base_interface,
        m_node_interfaces.topics_interface,
        m_node_interfaces.clock_interface,
        m_node_interfaces.logging_interface);


    m_on_commander_configured();
    m_base_state = CommanderBaseState::CONFIGURED;

    RCLCPP_INFO(m_logger, "Padflie Commander configured for %s", m_cf_prefix.c_str());
}

void 
PadflieCommanderBase::on_activate()
{
    if (m_base_state != CommanderBaseState::CONFIGURED) throw CommanderException("PadflieCommanderBase is not configured!");
    if (!m_hw_state_controller.is_charged()) throw CommanderException("Crazyflie is not charged!");
    if (!m_hw_state_controller.canfly()) throw CommanderException("Crazyflie cannot fly!");
    Eigen::Vector3d position;
    if (!m_padflie_tf->get_cf_position(position)) throw CommanderException("Crazyflie position is not available!");

    m_activate_commander();

    m_hardware_actor = std::make_shared<HardwareActor>(
        m_node_interfaces.base_interface,
        m_node_interfaces.topics_interface,
        m_node_interfaces.graph_interface,
        m_node_interfaces.services_interface,
        m_node_interfaces.timers_interface,
        m_node_interfaces.clock_interface,
        m_node_interfaces.logging_interface,
        m_cf_prefix,
        m_padflie_tf);

    m_create_control_interface();

    m_on_commander_activated();
    m_base_state = CommanderBaseState::ACTIVATED;
    RCLCPP_INFO(m_logger, "Padflie Commander activated for %s", m_cf_prefix.c_str());
}

void 
PadflieCommanderBase::on_deactivate(bool force)
{ 
    if (m_base_state != CommanderBaseState::ACTIVATED) throw CommanderException("PadflieCommanderBase is not activated, invalid transition!");

    m_remove_control_interface(); // First block all incomming commands

    m_deactivate_commander(force);
    
    m_hardware_actor.reset(); 
    m_hw_state_controller.reset_state();
    m_on_commander_deactivated();
    m_base_state = CommanderBaseState::CONFIGURED;
    RCLCPP_INFO(m_logger, "Padflie Commander deactivated for %s", m_cf_prefix.c_str());
}

void PadflieCommanderBase::m_on_state_callback()
{
}

void 
PadflieCommanderBase::m_handle_info_timer()
{
    padflies_interfaces::msg::PadflieInfo info_msg;
    info_msg.cf_prefix = m_cf_prefix;
    if (m_padflie_tf->get_cf_pose_stamped(m_hardware_actor->get_current_target_frame(), info_msg.pose)) 
         info_msg.pose_valid = true;
    Eigen::Vector3d position;
    if (m_padflie_tf->get_cf_position(position))
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

void PadflieCommanderBase::m_create_control_interface()
{
    auto sub_options = rclcpp::SubscriptionOptions();
    sub_options.callback_group = m_callback_group;

    auto pub_options = rclcpp::PublisherOptions();
    pub_options.callback_group = m_callback_group;

    m_takeoff_service = rclcpp::create_service<std_srvs::srv::Trigger>(
        m_node_interfaces.base_interface,
        m_node_interfaces.services_interface,
        m_prefix + "/takeoff",
        std::bind(&PadflieCommanderBase::m_handle_takeoff_command, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        rclcpp::ServicesQoS(),
        m_callback_group);

    m_land_service = rclcpp::create_service<std_srvs::srv::Trigger>(
        m_node_interfaces.base_interface,
        m_node_interfaces.services_interface,
        m_prefix + "/land",
        std::bind(&PadflieCommanderBase::m_handle_land_command, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
        rclcpp::ServicesQoS(),
        m_callback_group);

    m_send_target_sub = rclcpp::create_subscription<padflies_interfaces::msg::SendTarget>(
        m_node_interfaces.topics_interface,
        m_prefix + "/send_target", 10,
        std::bind(&PadflieCommanderBase::m_handle_send_target_command, this, std::placeholders::_1),
        sub_options);

    m_padflie_info_pub = rclcpp::create_publisher<padflies_interfaces::msg::PadflieInfo>(
        m_node_interfaces.topics_interface,
        m_prefix + "/info", 10, pub_options);
    m_padflie_info_timer = rclcpp::create_timer(
        m_node_interfaces.base_interface,
        m_node_interfaces.timers_interface,
        m_node_interfaces.clock_interface->get_clock(),
        std::chrono::milliseconds(100),
        std::bind(&PadflieCommanderBase::m_handle_info_timer, this),
        m_callback_group
    );
}

void PadflieCommanderBase::m_remove_control_interface()
{
    m_takeoff_service.reset();
    m_land_service.reset();
    m_send_target_sub.reset();

    m_padflie_info_timer->cancel();
    m_padflie_info_timer.reset();
    m_padflie_info_pub.reset();
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
