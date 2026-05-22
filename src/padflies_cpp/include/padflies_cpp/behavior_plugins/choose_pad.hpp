#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "padflies_cpp/pad_client.hpp"
#include "padflies_cpp/pad_client_factory.hpp"

namespace padflies_cpp::behavior_plugins
{
    class ChoosePad : public BT::SyncActionNode
    {
    public: 
    ChoosePad(
      const std::string& name,
      const BT::NodeConfig& config,
      rclcpp::Logger logger,
      std::shared_ptr<PadClientFactory> pad_client_factory);
    
    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::shared_ptr<PadClient>>("pad_client")
        };
    }


    };

} // namespace padflies_cpp::behavior_plugins