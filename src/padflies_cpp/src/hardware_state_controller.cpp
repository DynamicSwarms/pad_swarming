#include "padflies_cpp/hardware_state_controller.hpp"


HardwareStateController::HardwareStateController(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_iface)
: m_param_iface(param_iface)
{
    param_iface->declare_parameter("battery_voltage_critical", rclcpp::ParameterValue(3.3));
    param_iface->declare_parameter("battery_voltage_empty", rclcpp::ParameterValue(3.44));
    param_iface->declare_parameter("battery_voltage_charged", rclcpp::ParameterValue(4.14));
}

void 
HardwareStateController::on_configure(
    const std::string & cf_prefix,
    std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node)
{
    m_callback_group = node->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group;

    m_battery_sub = node->create_subscription<crazyflie_interfaces::msg::GenericLogData>(
        cf_prefix + "/state",
        10,
        std::bind(&HardwareStateController::m_on_state_data, this, std::placeholders::_1),
        sub_opt);
}

void 
HardwareStateController::set_on_charged_callback(std::function<void()> callback)
{
    m_on_charged_callback = std::move(callback);
}


bool 
HardwareStateController::is_charged() const
{
    return m_battery_voltage >= m_get_battery_voltage_charged();
}

bool 
HardwareStateController::is_empty() const
{
    return m_battery_voltage <= m_get_battery_voltage_empty();
}

bool 
HardwareStateController::is_critical() const
{
    return m_battery_voltage <= m_get_battery_voltage_critical();
}

double 
HardwareStateController::get_battery_voltage() const
{
    return m_battery_voltage;
}

bool 
HardwareStateController::canfly() const
{
    return m_canfly;
}   
bool 
HardwareStateController::is_flying() const
{
    return m_is_flying;
}
bool        
HardwareStateController::is_tumbled() const
{
    return m_is_tumbled;
}

void 
HardwareStateController::m_on_state_data(
    const crazyflie_interfaces::msg::GenericLogData::SharedPtr msg)
{
    // Variables: "pm.vbat", "pm.chargeCurrent", "pm.state", "sys.canfly", "sys.isFlying", "sys.isTumbled"
    if (msg->values.size() == 6) { // Full state data
        m_battery_voltage = msg->values[0];
        m_battery_charge_current = msg->values[1];
        m_battery_charge_state = (int)msg->values[2];
        m_canfly = (bool)(int)msg->values[3];
        m_is_flying = (bool)(int)msg->values[4];
        m_is_tumbled = (bool)(int)msg->values[5];
    } else if (msg->values.size() == 1) { // Only battery voltage
        m_battery_voltage = msg->values[0];
        m_canfly = true;
        m_is_flying = true;
        m_is_tumbled = false;
    } else {
        RCLCPP_WARN(
            rclcpp::get_logger("HardwareStateController"),
            "Received unexpected number of values in state data: %zu", msg->values.size());
        return;
    }

    
    if (this->is_charged() && m_on_charged_callback) {
        m_on_charged_callback();
    }
}


double 
HardwareStateController::m_get_battery_voltage_charged() const
{
    return m_param_iface->get_parameter("battery_voltage_charged").as_double();
}

double 
HardwareStateController::m_get_battery_voltage_empty() const
{
    return m_param_iface->get_parameter("battery_voltage_empty").as_double();
}

double 
HardwareStateController::m_get_battery_voltage_critical() const
{
    return m_param_iface->get_parameter("battery_voltage_critical").as_double();
}