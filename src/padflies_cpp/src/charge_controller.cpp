#include "padflies_cpp/charge_controller.hpp"


ChargeController::ChargeController(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_iface)
    : m_battery_voltage(0.0)
, m_param_iface(param_iface)
{
    param_iface->declare_parameter("battery_voltage_empty", rclcpp::ParameterValue(3.44));
    param_iface->declare_parameter("battery_voltage_charged", rclcpp::ParameterValue(4.14));
}

void ChargeController::on_configure(
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
        std::bind(&ChargeController::on_battery_data, this, std::placeholders::_1),
        sub_opt);
}

void ChargeController::set_on_charged_callback(std::function<void()> callback)
{
    m_on_charged_callback = std::move(callback);
}


bool ChargeController::is_charged() const
{
    return m_battery_voltage >= get_battery_voltage_charged();
}

bool ChargeController::is_empty() const
{
    return m_battery_voltage <= get_battery_voltage_empty();
}

double ChargeController::get_battery_voltage() const
{
    return m_battery_voltage;
}


void ChargeController::on_battery_data(
    const crazyflie_interfaces::msg::GenericLogData::SharedPtr msg)
{
    // Variables: "pm.vbat", "pm.chargeCurrent", "pm.state", "sys.canfly", "sys.isFlying", "sys.isTumbled", "sys.armed"
    m_battery_voltage = msg->values[0];

    if (is_charged() && m_on_charged_callback) {
        m_on_charged_callback();
    }
}


double ChargeController::get_battery_voltage_charged() const
{
    return m_param_iface->get_parameter("battery_voltage_charged").as_double();
}

double ChargeController::get_battery_voltage_empty() const
{
    return m_param_iface->get_parameter("battery_voltage_empty").as_double();
}