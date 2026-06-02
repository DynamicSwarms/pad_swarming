#pragma once

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace padflies_cpp
{
    class IPadflieBehaviorPlugin
    {
    public:
        IPadflieBehaviorPlugin(rclcpp::Logger logger) 
        : m_logger(logger.get_child("IPadflieBehaviorPlugin"))
        {       }

        virtual ~IPadflieBehaviorPlugin() = default;

        virtual void registerNodes(BT::BehaviorTreeFactory & factory) = 0;
    
    private: 
        rclcpp::Logger m_logger;
    };
}