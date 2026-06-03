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
};

class ListOfPadInfos
{
public:
    void update(const pad_management_interfaces::msg::PadInfo::SharedPtr msg){
        std::lock_guard<std::mutex> lock(m_mutex);
        m_pad_infos[msg->node_name] = PadInfo{
            .node_name = msg->node_name,
            .pad_idle_target_service_name = msg->pad_idle_target_service_name,
            .pad_right_control_action_name = msg->pad_right_control_action_name,
            .pad_tf_names = msg->pad_tf_names
        };
    };

    void get_all_pad_infos(std::map<std::string, PadInfo> & pad_infos){
        std::lock_guard<std::mutex> lock(m_mutex);
        pad_infos = m_pad_infos;
    };
    

private:
    std::mutex m_mutex;
    std::map<std::string, PadInfo> m_pad_infos;
};

class PadflieBehaviors : public padflies_cpp::IPadflieBehaviorPlugin
{
public:
   PadflieBehaviors(padflies_cpp::NodeInterfacesBundle node_interfaces_bundle, rclcpp::Logger logger)
  : m_node_interfaces_bundle(node_interfaces_bundle)
  , m_logger(logger.get_child("PadflieBehaviors"))
  , m_list_of_pad_infos(std::make_shared<ListOfPadInfos>())
  {
    m_pad_info_subscription = rclcpp::create_subscription<pad_management_interfaces::msg::PadInfo>(
      m_node_interfaces_bundle.topics_interface,
      "pad_management/pad_info",
      rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&PadflieBehaviors::pad_info_callback, this, std::placeholders::_1)
    );
  }

    void pad_info_callback(const pad_management_interfaces::msg::PadInfo::SharedPtr msg)
    {
        m_list_of_pad_infos->update(msg);
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

    std::shared_ptr<rclcpp::Subscription<pad_management_interfaces::msg::PadInfo>> m_pad_info_subscription;
    std::shared_ptr<ListOfPadInfos> m_list_of_pad_infos;
};

}  // namespace padflie_behaviors_base

