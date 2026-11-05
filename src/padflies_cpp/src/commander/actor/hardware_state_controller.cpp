#include "padflies_cpp/commander/actor/hardware_state_controller.hpp"

#include <numbers>
#include <cmath>


HardwareStateController::HardwareStateController(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_iface)
: m_param_iface(param_iface)
{
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    auto range = rcl_interfaces::msg::FloatingPointRange();
    range.from_value = 2.8;
    range.to_value = 4.3;
    descriptor.floating_point_range.push_back(range);

    param_iface->declare_parameter("battery_voltage_critical", rclcpp::ParameterValue(3.3), descriptor);
    param_iface->declare_parameter("battery_voltage_empty", rclcpp::ParameterValue(3.44), descriptor);
    param_iface->declare_parameter("battery_voltage_charged", rclcpp::ParameterValue(4.14), descriptor);
}

void 
HardwareStateController::connect(
    const std::string & cf_prefix,
    const padflies_cpp::NodeInterfacesBundle & node_interfaces)
{
    m_clock = node_interfaces.clock_interface->get_clock();
    m_callback_group = node_interfaces.base_interface->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = m_callback_group;
    auto topics_interface = node_interfaces.topics_interface;

    m_battery_sub = rclcpp::create_subscription<crazyflie_interfaces::msg::LogDataGeneric>(
        topics_interface,
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

void
HardwareStateController::set_on_state_callback(std::function<void()> callback)
{
    m_on_state_callback = std::move(callback);
}


bool 
HardwareStateController::is_charged() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_battery_voltage >= m_get_battery_voltage_charged();
}

bool 
HardwareStateController::is_empty() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_battery_voltage <= m_get_battery_voltage_empty();
}

bool 
HardwareStateController::is_critical() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_battery_voltage <= m_get_battery_voltage_critical();
}

double 
HardwareStateController::get_battery_voltage() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_battery_voltage;
}

bool HardwareStateController::yaw_valid() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_yaw_valid;
}

double HardwareStateController::get_yaw() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_yaw;
}

bool 
HardwareStateController::canfly() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_canfly;
}   
bool 
HardwareStateController::is_flying() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_is_flying;
}
bool        
HardwareStateController::is_tumbled() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_is_tumbled;
}

void 
HardwareStateController::reset_state()
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    m_snapshot.can_fly = false;
    m_snapshot.flying = false;
    m_snapshot.tumbled = false;
    m_snapshot.received_at = rclcpp::Time(0, 0, RCL_ROS_TIME);
    m_battery_voltage = 0.0;
    m_battery_charge_current = 0.0;
    m_battery_charge_state = 0;
    m_canfly = false;
    m_is_flying = false;
    m_is_tumbled = true;
    m_yaw = 0.0;
    m_yaw_valid = false;
}

void 
HardwareStateController::m_on_state_data(
    const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg)
{
    // Malformed data must not renew the telemetry deadline.
    const auto count = msg->values.size();
    if (count == 1 || count == 6 || count == 7) {
        for (std::size_t i = 0; i < (count == 1 ? 1u : 6u); ++i) {
            if (!std::isfinite(msg->values[i])) return;
        }
    }
    {
        std::lock_guard<std::mutex> lock(m_state_mutex);
        // Variables: pm.vbat, pm.chargeCurrent, pm.state, sys.canfly,
        // sys.isFlying, sys.isTumbled, stateEstimate.yaw (degrees).
        if (msg->values.size() == 7 || msg->values.size() == 6) { // Full state data
            m_battery_voltage = msg->values[0];
            m_battery_charge_current = msg->values[1];
            m_battery_charge_state = (int)msg->values[2];
            m_canfly = (bool)(int)msg->values[3];
            m_is_flying = (bool)(int)msg->values[4];
            m_is_tumbled = (bool)(int)msg->values[5];
            if (msg->values.size() == 7) {
                m_yaw = msg->values[6] * std::numbers::pi / 180.0;
                m_yaw_valid = std::isfinite(m_yaw);
            } else {
                m_yaw_valid = false;
            }
        } else if (msg->values.size() == 1) { // Only battery voltage
            m_battery_voltage = msg->values[0];
            m_canfly = true;
            m_is_flying = true;
            m_is_tumbled = false;
            m_yaw_valid = false;
        } else {
            RCLCPP_WARN(
                rclcpp::get_logger("HardwareStateController"),
                "Received unexpected number of values in state data: %zu", msg->values.size());
            return;
        }

        m_snapshot = {m_canfly, m_is_flying, m_is_tumbled,
            m_snapshot.sequence + 1, m_clock->now()};
    }

    if (this->is_charged() && m_on_charged_callback) {
        m_on_charged_callback();
    }
    if (m_on_state_callback) {
        m_on_state_callback();
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

padflies_cpp::HardwareStateSnapshot HardwareStateController::snapshot() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_snapshot;
}
