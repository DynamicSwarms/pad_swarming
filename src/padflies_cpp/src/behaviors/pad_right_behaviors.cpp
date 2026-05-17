#include "behaviortree_cpp/bt_factory.h"
#include "padflies_cpp/behaviors/pad_right_behaviors.hpp"



class SendRequest : public BT::SyncActionNode
{
public: 
    SendRequest(
        const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Logger logger)
    : BT::SyncActionNode(name, config)
    , m_logger(logger.get_child(name))
    {}
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadRightClient>>("client"),
            BT::InputPort<uint8_t>("action")
        };
    }    
    BT::NodeStatus tick() override
    {
        std::shared_ptr<PadRightClient> client;
        uint8_t action;
        if (!getInput("client", client) || !getInput("action", action))
        {
            RCLCPP_ERROR(getLogger(), "Error getting input ports!");
            return BT::NodeStatus::FAILURE;
        }

        if (m_pad_right_client->is_action_server_available()) {
            m_pad_right_client->send_request(m_action);
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::FAILURE;
        }
    }
private: 
    rclcpp::Logger m_logger;
}

class CancelRequest : public BT::SyncActionNode
{
public: 
    CancelRequest(
        const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Logger logger)
    : BT::SyncActionNode(name, config)
    , m_logger(logger.get_child(name))
    {}
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadRightClient>>("client")
        };
    }    
    BT::NodeStatus tick() override
    {        
        std::shared_ptr<PadRightClient> client;
        if (!getInput("client", client))
        { 
            RCLCPP_ERROR(getLogger(), "Error getting input port [client]!");
            return BT::NodeStatus::FAILURE;
        }
        m_pad_right_client->cancel_goal();
        return BT::NodeStatus::SUCCESS;
    }
private: 
    rclcpp::Logger m_logger;
};


template<bool (PadRightClient::*ConditionFunc)() const>
class PadRightCondition : public BT::ConditionNode
{
public:
    PadRightCondition(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Logger logger)
    : BT::ConditionNode(name, config)
    , m_logger(logger.get_child(name))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadRightClient>>("client")
        };
    }

    BT::NodeStatus tick() override
    {
        std::shared_ptr<PadRightClient> client;

        if (!getInput("client", client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [client]");
            return BT::NodeStatus::FAILURE;
        }

        bool result = (client.get()->*ConditionFunc)();

        return result
            ? BT::NodeStatus::SUCCESS
            : BT::NodeStatus::FAILURE;
    }

private:
    rclcpp::Logger m_logger;
};

using GoalRespondedCondition = PadRightCondition<&PadRightClient::goal_responded>;
using GoalAcceptedCondition = PadRightCondition<&PadRightClient::goal_accepted>;
using HasRightCondition = PadRightCondition<&PadRightClient::has_right>;
using ReceivedResultCondition = PadRightCondition<&PadRightClient::received_result>;
using ResultSuccessCondition = PadRightCondition<&PadRightClient::result_success>;
