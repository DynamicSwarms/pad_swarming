#pragma once
#include "padflie_behaviors_base/visibility_control.h"
#include "padflies_cpp/I_padflie_behavior_plugin.hpp"

namespace padflie_behaviors_base
{

class PadflieBehaviorsBase : public padflies_cpp::IPadflieBehaviorPlugin
{
public:
   PadflieBehaviorsBase()
  : padflies_cpp::IPadflieBehaviorPlugin()
  {
  }

  void registerNodes(BT::BehaviorTreeFactory & factory) override; 


};

}  // namespace padflie_behaviors_base

