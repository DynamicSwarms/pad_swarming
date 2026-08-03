#pragma once

#include "behaviortree_cpp/bt_factory.h"
#include "class_loader/class_loader_core.hpp"
#include "rclcpp/rclcpp.hpp"

#include "padflies_cpp/commander/actor/hardware_actor.hpp"
#include "padflies_cpp/commander/actor/hardware_state_controller.hpp"
#include "padflies_cpp/commander/command/routine/routine_result.hpp"
#include "padflies_cpp/node_interfaces_bundle.hpp"
#include "padflies_cpp/commander/padflie_tf.hpp"
#include "pad_management_interfaces/msg/site_info.hpp"

namespace padflies_cpp
{
class IBehaviorPlugin
{
public:
  virtual ~IBehaviorPlugin() = default;

  virtual BT::Tree getTree(
    BT::BehaviorTreeFactory & factory,
    std::shared_ptr<HardwareActor> hardware_actor,
    std::shared_ptr<HardwareStateController> hardware_state_controller,
    std::shared_ptr<PadflieTF> padflie_tf,
    const pad_management_interfaces::msg::SiteInfo & site_info) = 0;

  virtual RoutineResultClassifier getResultClassifier() const = 0;
};

class ITakeoffPlugin : public IBehaviorPlugin
{
public:
  virtual ~ITakeoffPlugin() = default;
};

class ILandingPlugin : public IBehaviorPlugin
{
public:
  virtual ~ILandingPlugin() = default;
};
}  // namespace padflies_cpp

namespace class_loader
{
template<>
struct InterfaceTraits<padflies_cpp::ITakeoffPlugin>
{
  using constructor_parameters = ConstructorParameters<padflies_cpp::NodeInterfacesBundle, rclcpp::Logger>;
};

template<>
struct InterfaceTraits<padflies_cpp::ILandingPlugin>
{
  using constructor_parameters = ConstructorParameters<padflies_cpp::NodeInterfacesBundle, rclcpp::Logger>;
};
}  // namespace class_loader
