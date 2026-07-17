#pragma once

#include "padflies_cpp/I_padflie_behavior_plugin.hpp"
#include "padflies_cpp/node_interfaces_bundle.hpp"
#include "rclcpp/rclcpp.hpp"

#include "pad_management_interfaces/msg/pad_info.hpp"

namespace padflie_behaviors
{

struct PadInfo
{
    std::string node_name;
    std::string pad_idle_target_service_name;
    std::string pad_right_control_action_name;
    std::vector<std::string> pad_tf_names;
    bool available;
    rclcpp::Duration wait_time{0, 0};
    uint8_t charging_speed;
};

class PadInfos
{
public:
    PadInfos(std::string initial_pad) 
    : m_current_pad_name(initial_pad)
    {

    }

    void update(const pad_management_interfaces::msg::PadInfo::SharedPtr msg){
        std::lock_guard<std::mutex> lock(m_mutex);
        m_pad_infos[msg->node_name] = PadInfo{
            .node_name = msg->node_name,
            .pad_idle_target_service_name = msg->pad_idle_target_service_name,
            .pad_right_control_action_name = msg->pad_right_control_action_name,
            .pad_tf_names = msg->pad_tf_names,
            .available = msg->available,
            .wait_time = msg->wait_time,
            .charging_speed = msg->charging_speed
        };

        RCLCPP_DEBUG(rclcpp::get_logger("PadInfos"), "Updated pad info for node: %s, available: %s, pad_tf_names size: %zu", 
                    msg->node_name.c_str(), 
                    msg->available ? "true" : "false", 
                    msg->pad_tf_names.size());
    };

    void get_all_pad_infos(std::map<std::string, PadInfo> & pad_infos){
        std::lock_guard<std::mutex> lock(m_mutex);
        pad_infos = m_pad_infos;
    };

    std::string get_current_pad_name(){
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_current_pad_name;
    };

    void set_current_pad_name(const std::string & new_pad_name){
        std::lock_guard<std::mutex> lock(m_mutex);
        m_current_pad_name = new_pad_name;
    };
    

private:
    std::mutex m_mutex;
    std::map<std::string, PadInfo> m_pad_infos;

    std::string m_current_pad_name;
};

class PadflieBehaviors : public padflies_cpp::IPadflieBehaviorPlugin
{
public:
    PadflieBehaviors(padflies_cpp::NodeInterfacesBundle node_interfaces_bundle, rclcpp::Logger logger)
    : m_node_interfaces_bundle(node_interfaces_bundle)
    , m_logger(logger.get_child("PadflieBehaviors"))
    {
        m_callback_group = m_node_interfaces_bundle.base_interface->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        auto sub_options = rclcpp::SubscriptionOptions();
        sub_options.callback_group = m_callback_group;
        m_pad_info_subscription = rclcpp::create_subscription<pad_management_interfaces::msg::PadInfo>(
            m_node_interfaces_bundle.topics_interface,
            "pad_management/pad_info",
            rclcpp::QoS(10).reliable().transient_local(),
            std::bind(&PadflieBehaviors::pad_info_callback, this, std::placeholders::_1),
            sub_options
        );

        auto param_overrides =  node_interfaces_bundle.parameters_interface->get_parameter_overrides();
        for (auto &  [name, value] : param_overrides) RCLCPP_DEBUG(m_logger, "Parameter override: %s = %s", name.c_str(), rclcpp::to_string(value).c_str());

        //p_initial_pad = node_interfaces_bundle.parameters_interface->declare_parameter("initial_pad", rclcpp::ParameterValue(""), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<std::string>();
        //RCLCPP_INFO(m_logger, "Initial pad parameter declared with value: '%s'", p_initial_pad.c_str());

        if (!node_interfaces_bundle.parameters_interface->has_parameter("initial_pad"))
        {
            p_initial_pad = node_interfaces_bundle.parameters_interface->declare_parameter(
                "initial_pad", 
                rclcpp::ParameterValue(""), // Leaves type unset initially
                rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)
            ).get<std::string>();
        }
        RCLCPP_INFO(m_logger, "Initial pad parameter is: '%s'", p_initial_pad.c_str());

        if (!node_interfaces_bundle.parameters_interface->has_parameter("slow_only"))
        {
            p_slow_only = node_interfaces_bundle.parameters_interface->declare_parameter(
                "slow_only", 
                rclcpp::ParameterValue(false), // Leaves type unset initially
                rcl_interfaces::msg::ParameterDescriptor()
            ).get<bool>();
        }

        m_param_callback_handle = 
            m_node_interfaces_bundle.parameters_interface->add_on_set_parameters_callback(std::bind(&PadflieBehaviors::m_set_parameters_callback, this, std::placeholders::_1));

        m_list_of_pad_infos = std::make_shared<PadInfos>(p_initial_pad);
    }

    ~PadflieBehaviors() 
    {
        RCLCPP_INFO(m_logger, "PadflieBehaviors destructor called.");        
        m_param_callback_handle.reset();
        //m_node_interfaces_bundle.parameters_interface->undeclare_parameter("initial_pad");
    }

    void pad_info_callback(const pad_management_interfaces::msg::PadInfo::SharedPtr msg)
    {
        m_list_of_pad_infos->update(msg);
    }

    rcl_interfaces::msg::SetParametersResult 
        m_set_parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
    {

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto & param : parameters)
        {
            if (param.get_name() == "slow_only")
            {
                RCLCPP_INFO(m_logger, "Parameter 'slow_only' changed to '%s'.", param.as_bool() ? "true" : "false");
                p_slow_only = param.as_bool();
            }
        }

        return result;
    }

  BT::Tree getTakeoffTree(BT::BehaviorTreeFactory & factory, 
      std::shared_ptr<HardwareActor> hardware_actor,
      std::shared_ptr<PadflieTF> padflie_tf,
      std::shared_ptr<PadExecuteServer> pad_execute_server,
      std::shared_ptr<PadClientFactory> pad_client_factory) override;
  BT::Tree getLandTree(BT::BehaviorTreeFactory & factory, 
      std::shared_ptr<HardwareActor> hardware_actor,
      std::shared_ptr<PadflieTF> padflie_tf,
      std::shared_ptr<PadExecuteServer> pad_execute_server,
      std::shared_ptr<PadClientFactory> pad_client_factory) override;
private: 
    padflies_cpp::NodeInterfacesBundle m_node_interfaces_bundle;
    rclcpp::Logger m_logger;

    std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
    std::shared_ptr<rclcpp::Subscription<pad_management_interfaces::msg::PadInfo>> m_pad_info_subscription;
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> m_param_callback_handle; 

    std::shared_ptr<PadInfos> m_list_of_pad_infos;

    

    std::string p_initial_pad;
    bool p_slow_only;
};

}  // namespace padflie_behaviors_base

