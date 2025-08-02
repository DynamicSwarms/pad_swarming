#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "padflies_interfaces/msg/padflie_info.hpp"
#include "crazyflie_interfaces/msg/generic_log_data.hpp"
using std::placeholders::_1;

class ChargeController
{
    public:
        ChargeController(
            rclcpp::node_interfaces::NodeParametersInterface::SharedPtr param_iface
        );

        void on_configure(    
            const std::string & cf_prefix,
            std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node
        );

        void set_on_charged_callback(std::function<void()> callback);
        bool is_charged() const;
        bool is_empty() const;
        bool is_critical() const;
        double get_battery_voltage() const;

    private:
        void on_battery_data(
            const crazyflie_interfaces::msg::GenericLogData::SharedPtr msg
        );

        double get_battery_voltage_charged() const;
        double get_battery_voltage_empty() const;
        double get_battery_voltage_critical() const;

    private: 
        double m_battery_voltage;

        rclcpp::node_interfaces::NodeParametersInterface::SharedPtr m_param_iface;

        rclcpp::CallbackGroup::SharedPtr m_callback_group;
        rclcpp::Subscription<crazyflie_interfaces::msg::GenericLogData>::SharedPtr m_battery_sub;
        
        std::function<void()> m_on_charged_callback;
};