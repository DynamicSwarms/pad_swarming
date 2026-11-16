#pragma once

#include "rclcpp/rclcpp.hpp"
#include <mutex>
#include "padflies_cpp/commander/actor/hardware_state_snapshot.hpp"
#include "padflies_cpp/node_interfaces_bundle.hpp"

#include "crazyflie_interfaces/msg/log_data_generic.hpp"
using std::placeholders::_1;

class HardwareStateController
{
    public:
        HardwareStateController(
            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_iface
        );

        void connect(    
            const std::string & cf_prefix,
            const padflies_cpp::NodeInterfacesBundle & node_interfaces
        );

        void set_on_charged_callback(std::function<void()> callback);
        void set_on_state_callback(std::function<void()> callback);
        bool is_charged() const;
        bool is_empty() const;
        bool is_critical() const;
        double get_battery_voltage() const;
        // Estimated SoC in percent [0, 100]; NaN before telemetry or while
        // compensated flight calibration is unavailable. Ground readings may
        // include charging influence. With compensation disabled, uses raw voltage.
        double get_soc() const;
        bool yaw_valid() const;
        double get_yaw() const;

        bool canfly() const;
        bool is_flying() const;
        bool is_tumbled() const;

        void reset_state();
        padflies_cpp::HardwareStateSnapshot snapshot() const;

    private:
        void m_on_state_data(
            const crazyflie_interfaces::msg::LogDataGeneric::SharedPtr msg
        );

        double m_comparison_voltage() const;
        double m_get_battery_voltage_charged() const;
        double m_get_battery_voltage_empty() const;
        double m_get_battery_voltage_critical() const;

    private: 
        rclcpp::Clock::SharedPtr m_clock;
        mutable std::mutex m_state_mutex;
        padflies_cpp::HardwareStateSnapshot m_snapshot;
        bool m_compensated = false;
        double m_max_drop = 0.6;
        double m_ground_voltage = 0.0;
        bool m_ground_voltage_valid = false;
        unsigned m_flying_samples = 0;
        double m_second_flying_voltage = 0.0;
        double m_drop = 0.0;
        bool m_drop_valid = false;
        double m_battery_voltage = 0.0;
        double m_battery_charge_current = 0.0;
        int m_battery_charge_state = 0;
        bool m_canfly = false;
        bool m_is_flying = false;
        bool m_is_tumbled = false;
        double m_yaw = 0.0;
        bool m_yaw_valid = false;

        rclcpp::node_interfaces::NodeParametersInterface::SharedPtr m_param_iface;

        rclcpp::CallbackGroup::SharedPtr m_callback_group;
        rclcpp::Subscription<crazyflie_interfaces::msg::LogDataGeneric>::SharedPtr m_battery_sub;
        
        std::function<void()> m_on_charged_callback;
        std::function<void()> m_on_state_callback;
};
