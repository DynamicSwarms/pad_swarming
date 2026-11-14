#include "padflies_cpp/commander/actor/hardware_state_controller.hpp"

#include <numbers>
#include <cmath>
#include <stdexcept>
#include <array>
#include <algorithm>
#include <limits>

// Typical takeoff sag (4.2 V resting to 3.9 V flying) underlying legacy thresholds.
#define BATTERY_REFERENCE_DROP_V 0.30

namespace {
// Values reported by the custom Crazyflie firmware through pm.state.
enum class ChargeState : int {
    Charged = 2,
    ConstantVoltageCharging = 6,
};
}  // namespace


HardwareStateController::HardwareStateController(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_iface)
: m_param_iface(param_iface)
{
    m_compensated = param_iface->declare_parameter(
        "battery_compensation", rclcpp::ParameterValue(false)).get<bool>();
    m_max_drop = param_iface->declare_parameter(
        "battery_drop_max", rclcpp::ParameterValue(0.6)).get<double>();
    if (!std::isfinite(m_max_drop) || m_max_drop < 0.0) {
        throw std::invalid_argument("battery_drop_max must be finite and nonnegative");
    }
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
    if (m_compensated) {
        const auto charge_state = static_cast<ChargeState>(m_battery_charge_state);
        const bool charging_complete = charge_state == ChargeState::Charged;
        const bool in_cv_phase = charge_state == ChargeState::ConstantVoltageCharging;
        return m_battery_voltage > m_get_battery_voltage_charged() &&
            (in_cv_phase || charging_complete);
    }
    return m_battery_voltage >= m_get_battery_voltage_charged();
}

bool 
HardwareStateController::is_empty() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_comparison_voltage() <= m_get_battery_voltage_empty();
}

bool 
HardwareStateController::is_critical() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    return m_comparison_voltage() <= m_get_battery_voltage_critical();
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
    m_ground_voltage_valid = false;
    m_drop_valid = false;
    m_flying_samples = 0;
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

        if (m_compensated && count != 1) {
            if (!m_is_flying) {
                m_ground_voltage = m_battery_voltage;
                m_ground_voltage_valid = true;
                m_flying_samples = 0;
                m_drop_valid = false;
            } else if (m_flying_samples < 3) {
                ++m_flying_samples;
                if (m_flying_samples == 2) m_second_flying_voltage = m_battery_voltage;
                if (m_flying_samples == 3 && m_ground_voltage_valid) {
                    const double drop = m_ground_voltage -
                        (m_second_flying_voltage + m_battery_voltage) / 2.0;
                    // Reject implausible estimates; use legacy thresholds for this flight.
                    m_drop_valid = drop >= 0.0 && drop <= m_max_drop;
                    if (m_drop_valid) m_drop = drop;
                }
            }
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

// Caller holds m_state_mutex. Return voltage in the reference-load domain.
double HardwareStateController::m_comparison_voltage() const
{
    if (m_compensated && m_is_flying && m_drop_valid) {
        return m_battery_voltage + m_drop - BATTERY_REFERENCE_DROP_V;
    }
    return m_battery_voltage;
}


double HardwareStateController::get_soc() const
{
    std::lock_guard<std::mutex> lock(m_state_mutex);
    if (m_battery_voltage <= 0.0 ||
        (m_compensated && m_is_flying && !m_drop_valid)) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Approximate open-circuit curve digitized from thesis Figure 5.6b,
    // attributed there to Plett (2004), reference [93]. Entries are 5% apart.
    static constexpr std::array<double, 21> voltage_curve{
        3.0000, 3.6234, 3.7596, 3.8100, 3.8292, 3.8580, 3.8880,
        3.9066, 3.9300, 3.9480, 3.9660, 3.9840, 3.9960, 4.0080,
        4.0200, 4.0320, 4.0440, 4.0620, 4.0884, 4.1142, 4.2000
    };
    // SoC needs estimated open-circuit voltage, not the reference-load
    // voltage used for legacy empty/critical thresholds (no 0.30 V subtraction).
    const double voltage = m_battery_voltage +
        ((m_compensated && m_is_flying && m_drop_valid) ? m_drop : 0.0);
    if (voltage <= voltage_curve.front()) return 0.0;
    if (voltage >= voltage_curve.back()) return 100.0;
    const auto upper = std::lower_bound(voltage_curve.begin(), voltage_curve.end(), voltage);
    const auto index = static_cast<std::size_t>(upper - voltage_curve.begin());
    const double fraction = (voltage - voltage_curve[index - 1]) /
        (voltage_curve[index] - voltage_curve[index - 1]);
    return 5.0 * (static_cast<double>(index - 1) + fraction);
}
