#include "behaviortree_cpp/bt_factory.h"

#include "padflies_cpp/pad_execute_server.hpp"
#include "padflies_cpp/pad_right_client.hpp"

class GetPadRight : public BT::SatefulActionNode
{
public: 
    GetPadRight(
        const std::string& name, 
        const BT::NodeConfig& config,
        std::shared_ptr<PadExecuteServer> server,
        rclcpp::Logger logger)
    : BT::SatefulActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_server(server)
    {

    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadRightClient>>("pad_right_client")
            BT::InputPort<uint8_t>("action")
        };
    }
    
    BT::NodeStatus onStart() override
    {
        if (!getInput("pad_right_client", m_pad_right_client))
        {
            RCLCPP_ERROR(getLogger(), "Error getting input port [pad_right_client]!");
            return BT::NodeStatus::FAILURE;
        }

        if (!getInput("action", m_action))
        {
            RCLCPP_ERROR(getLogger(), "Error getting input port [action]!");
            return BT::NodeStatus::FAILURE;
        }

        std::string pad_name = m_pad_right_client->get_pad_name();
        m_pad_execute_server->set_selected_pad_name(pad_name);

        if (m_pad_right_client->is_action_server_available()) {
            m_pad_right_client->send_request(m_action);
            return BT::NodeStatus::RUNNING;
        } else {
            RCLCPP_ERROR(m_logger, "PadRight action server not available!");
            return BT::NodeStatus::FAILURE;
        }
    }

    BT::NodeStatus onRunning() override
    {
        if (!m_pad_right_client->goal_responded()) return BT::NodeStatus::RUNNING;
        if (!m_pad_right_client->goal_accepted()) {
            RCLCPP_ERROR(m_logger, "PadRight goal rejected!");
            return BT::NodeStatus::FAILURE;
        }

        if (m_pad_right_client->received_result())
        {
            RCLCPP_INFO(m_logger, "PadRight result received before doing anything. Failure!");
            return BT::NodeStatus::FAILURE;
        }

        if (m_pad_right_client->has_right()) {
            if (m_pad_execute_server->goal_received()) {
                if (m_pad_execute_server->goal_cancelled()) {
                    RCLCPP_INFO(m_logger, "PadRight acquired but Execute goal was cancelled!");
                    return BT::NodeStatus::FAILURE;
                } else 
                {
                    RCLCPP_INFO(m_logger, "PadRight acquired and Execute goal received!");
                    return BT::NodeStatus::SUCCESS;
                }
            } else {
                RCLCPP_INFO(m_logger, "PadRight acquired but still waiting for Execute goal...");
                return BT::NodeStatus::RUNNING;
            }
        } else {
            RCLCPP_INFO(m_logger, "Still waiting to acquire PadRight...");
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        if (m_pad_right_client) {
            m_pad_right_client->cancel_goal();
        }
        RCLCPP_INFO(m_logger, "GetPadRight halted, goal cancelled if it was sent.");
    }
private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;

    std::shared_ptr<PadRightClient> m_pad_right_client;
    uint8_t m_action;
};

class HoldPadRight : public BT::StatefulActionNode
{
public: 
    HoldPadRight(
        const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Logger logger,
        std::shared_ptr<PadExecuteServer> server)
    : BT::StatefulActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_pad_execute_server(server)
    {

    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadRightClient>>("pad_right_client")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!getInput("pad_right_client", m_pad_right_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_right_client]!");
            return BT::NodeStatus::FAILURE;
        }

        if (!m_pad_right_client->has_right() || 
            m_pad_right_client->received_result())
        {
            RCLCPP_ERROR(m_logger, "Cannot hold PadRight if we don't have, or if its already done!");
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus onRunning() override
    {
        if (m_pad_execute_server->result_sent()) 
        {
            RCLCPP_INFO(m_logger, "Execute goal result sent, stopping to hold PadRight.");
            if (!m_pad_right_client->received_result()) return BT::NodeStatus::RUNNING;
            if (m_pad_right_client->result_success()) {
                RCLCPP_INFO(m_logger, "Execute goal finished successfully and PadRight result success!");
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_ERROR(m_logger, "Execute goal finished cleanly, but Pad reported failure!");
                return BT::NodeStatus::FAILURE;
            } 
        }
        
        
        if (!m_pad_right_client->has_right() ||
            m_pad_right_client->received_result())
        {
            RCLCPP_ERROR(m_logger, "Cannot hold PadRight if we don't have, or if its already done!");
            return BT::NodeStatus::FAILURE;
        }

        
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        if (m_pad_right_client) {
            m_pad_right_client->cancel_goal();
        }
        RCLCPP_INFO(m_logger, "HoldPadRight halted, goal cancelled if it was sent.");
    }

private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;

    std::shared_ptr<PadRightClient> m_pad_right_client;
};

class ReleasePadRight : public BT::ActionNodeBase
{
public:
    ReleasePadRight(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Logger logger)
    : BT::ActionNodeBase(name, config)
    , m_logger(logger.get_child(name))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadRightClient>>("pad_right_client")
        };
    }

    BT::NodeStatus tick() override
    {
        std::shared_ptr<PadRightClient> client;
        if (!getInput("pad_right_client", client))
        { 
            RCLCPP_ERROR(m_logger, "Error getting input port [client]!");
            return BT::NodeStatus::FAILURE;
        }
     
        RCLCPP_INFO(m_logger, "Releasing PadRight...");
        client->cancel_goal();
        return BT::NodeStatus::SUCCESS;
    }

private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<PadRightClient> m_pad_right_client;
};

#include "Eigen/Dense"
#include "padflies_cpp/hardware_actor.hpp"


class LandRoutine : public BT::StatefulActionNode
{
public: 
    LandRoutine(
        const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Logger logger, 
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface),
        std::shared_ptr<HardwareActor> hardware_actor, 
        std::shared_ptr<PadExecuteServer> pad_execute_server,
    : BT::StatefulActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_clock(node_clock_interface->get_clock())
    , m_hardware_actor(hardware_actor)
    , m_pad_execute_server(pad_execute_server)
    {
        m_state = LandState::INIT;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadRightClient>>("pad_right_client")
        };
    }

    BT::NodeStatus onStart() {
        if (!getInput("pad_right_client", m_pad_right_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_right_client]!");
            return BT::NodeStatus::FAILURE;
        }

        m_state = LandState::INIT;
        m_phase_start_time = m_clock->now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        // TODO: send feedback

        if (m_phase_start_time + m_phase_durations[m_state] > m_clock->now()) 
            return BT::NodeStatus::RUNNING;
        
        m_phase_start_time = m_clock->now();

        switch (m_state) {
            case LandState::INIT:
                RCLCPP_INFO(m_logger, "Starting land routine...");
                m_hardware_actor->go_to(0.5, 0.0, 5.0);
                m_state = LandState::PHASE1;
                break;
            case LandState::PHASE1:
                RCLCPP_INFO(m_logger, "Phase 1: Moving to landing position...");
                m_hardware_actor->go_to(0.2, 0.0, 3.0);
                m_state = LandState::PHASE2;
                break;
            case LandState::PHASE2:
                RCLCPP_INFO(m_logger, "Phase 2: Final descent...");
                m_hardware_actor->land(0.0, 0.0, 2.0);
                m_state = LandState::PHASE3;
                break;
            case LandState::PHASE3:
                RCLCPP_INFO(m_logger, "Phase 3: Touchdown, waiting for PadRight result...");
                m_state = LandState::DONE;
                break;
            case LandState::DONE:
                // Send out PadExecute Result with success.

                m_state = LandState::DONE; // stay in DONE state
                break;
        }
    }

    void onHalted() {
        RCLCPP_INFO(m_logger, "LandRoutine halted, stopping the drone. What should happen here?");
    }




private: 
    rclcpp::Logger m_logger;
    rclcpp::Clock::SharedPtr m_clock;
    std::shared_ptr<HardwareActor> m_hardware_actor;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;  
    std::shared_ptr<PadRightClient> m_pad_right_client;

    enum class LandState {
        INIT,
        PHASE1,
        PHASE2,
        PHASE3, 
        DONE 
    } 
    LandState m_state = LandState::INIT;
    std::map<LandState, rclcpp::Duration> m_phase_durations = {
        {LandState::INIT, rclcpp::Duration(0s)},
        {LandState::PHASE1, rclcpp::Duration(5s)},
        {LandState::PHASE2, rclcpp::Duration(2s)},
        {LandState::PHASE3, rclcpp::Duration(1s)},
        {LandState::DONE, rclcpp::Duration(0s)}
    };
    rclcpp::Time m_phase_start_time;
    

};