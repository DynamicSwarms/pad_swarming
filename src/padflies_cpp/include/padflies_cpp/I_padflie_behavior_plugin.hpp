#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "class_loader/class_loader_core.hpp"

#include "padflies_cpp/hardware_actor.hpp"
#include "padflies_cpp/padflie_tf.hpp"
#include "padflies_cpp/pad_execute_server.hpp"
#include "padflies_cpp/pad_client_factory.hpp"
#include "padflies_cpp/node_interfaces_bundle.hpp"

namespace padflies_cpp
{
    class IPadflieBehaviorPlugin
    {
    public:
        virtual ~IPadflieBehaviorPlugin() = default;

        virtual BT::Tree getTakeoffTree(BT::BehaviorTreeFactory & factory, 
            std::shared_ptr<HardwareActor> hardware_actor,
            std::shared_ptr<PadflieTF> padflie_tf,
            std::shared_ptr<PadExecuteServer> pad_execute_server,
            std::shared_ptr<PadClientFactory> pad_client_factory) = 0;
        virtual BT::Tree getLandTree(BT::BehaviorTreeFactory & factory, 
            std::shared_ptr<HardwareActor> hardware_actor,
            std::shared_ptr<PadflieTF> padflie_tf,
            std::shared_ptr<PadExecuteServer> pad_execute_server,
            std::shared_ptr<PadClientFactory> pad_client_factory) = 0;
    };
}

namespace class_loader
{
template<>
struct InterfaceTraits<padflies_cpp::IPadflieBehaviorPlugin>
{
    using constructor_parameters = ConstructorParameters<padflies_cpp::NodeInterfacesBundle, rclcpp::Logger>;
};
} 