#include "behaviortree_cpp/bt_factory.h"
#include "padflies_cpp/pad_execute_server.hpp"

template<void (PadExecuteServer::*ActionFunc)(uint8_t)>
class PadExecuteAction : public BT::SyncActionNode
{
public: 
    PadExecuteAction(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<PadExecuteServer> server,
        rclcpp::Logger logger)
    : BT::SyncActionNode(name, config)
    , m_server(server)
    , m_logger(logger.get_child(name))
    {
        if (!m_server) {
            throw std::runtime_error("PadExecuteServer not initialized");
        }
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<uint8_t>("status", "Status to send in the feedback")
        };
    }

    BT::NodeStatus tick() override
    {
        uint8_t status;
        if (!getInput("status", status)) {
            RCLCPP_ERROR(m_logger, "Error getting input port [status]!");
            return BT::NodeStatus::FAILURE;
        }

        (m_server.get()->*ActionFunc)(status);
        return BT::NodeStatus::SUCCESS;
    }
private: 
    std::shared_ptr<PadExecuteServer> m_server;
    rclcpp::Logger m_logger;
};

using PadExecuteSendFeedback = PadExecuteAction<&PadExecuteServer::send_feedback>;
using PadExecuteSendResult = PadExecuteAction<&PadExecuteServer::send_result>;

template<bool (PadExecuteServer::*ConditionFunc)() const>
class PadExecuteCondition : public BT::ConditionNode
{public:
    PadExecuteCondition(
        const std::string& name,
        const BT::NodeConfig& config,
        std::shared_ptr<PadExecuteServer> server,
        rclcpp::Logger logger)
    : BT::ConditionNode(name, config)
    , m_server(server)
    , m_logger(logger.get_child(name))
    {
        if (!m_server) {
            throw std::runtime_error("PadExecuteServer not initialized");
        }
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        bool result = (m_server.get()->*ConditionFunc)();
        return result ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
private:
    std::shared_ptr<PadExecuteServer> m_server;
    rclcpp::Logger m_logger;
};

using PadExecuteGoalReceivedCondition = PadExecuteCondition<&PadExecuteServer::goal_received>;
using PadExecuteGoalCancelledCondition = PadExecuteCondition<&PadExecuteServer::goal_cancelled>;
