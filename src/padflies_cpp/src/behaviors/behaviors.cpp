#include "behaviortree_cpp/bt_factory.h"

#include "padflies_cpp/pad_execute_server.hpp"
#include "padflies_cpp/pad_client.hpp"
#include "Eigen/Dense"
#include "padflies_cpp/hardware_actor.hpp"
using namespace std::chrono_literals;
class ChoosePad : public BT::SyncActionNode
{
public: 
    ChoosePad(
        const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Logger logger,
        std::shared_ptr<PadClientFactory> pad_client_factory)
    : BT::SyncActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_pad_client_factory(pad_client_factory)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::shared_ptr<PadClient>>("pad_client")
        };
    }

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(m_logger, "Choosing a pad and creating PadClient...");
        std::shared_ptr<PadClient> pad_client = m_pad_client_factory->create_pad_client("megapad");
    
        setOutput("pad_client", pad_client);
        return BT::NodeStatus::SUCCESS;
    }
    
private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<PadClientFactory> m_pad_client_factory;
};

class GetPadRight : public BT::StatefulActionNode
{
public: 
    GetPadRight(
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
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client"),
            BT::InputPort<std::string>("action")
        };
    }
    
    BT::NodeStatus onStart() override
    {
        RCLCPP_INFO(m_logger, "GetPadRight started, trying to acquire right and waiting for Execute goal...");
        if (!getInput("pad_client", m_pad_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_client]!");
            return BT::NodeStatus::FAILURE;
        }

        std::string action_string; 
        if (!getInput("action", action_string))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [action]!");
            return BT::NodeStatus::FAILURE;
        }
        if (action_string == "TAKEOFF") m_action = pad_management_interfaces::action::PadRightControl::Goal::ACTION_TAKEOFF;
        else if (action_string == "LAND") m_action = pad_management_interfaces::action::PadRightControl::Goal::ACTION_LAND;
        else if (action_string == "PLACE_ON") m_action = pad_management_interfaces::action::PadRightControl::Goal::ACTION_PLACE_ON;
        else if (action_string == "REMOVE") m_action = pad_management_interfaces::action::PadRightControl::Goal::ACTION_REMOVE;
        else {
            RCLCPP_ERROR(m_logger, "Invalid action string [action]!");
            return BT::NodeStatus::FAILURE;
        }
        


        std::string pad_name = m_pad_client->get_pad_name();
        m_pad_execute_server->set_selected_pad_name(pad_name);

        if (m_pad_client->is_action_server_available()) {
            m_pad_client->send_request(m_action);
            return BT::NodeStatus::RUNNING;
        } else {
            RCLCPP_ERROR(m_logger, "PadRight action server not available!");
            return BT::NodeStatus::FAILURE;
        }
    }



    BT::NodeStatus onRunning() override
    {
        if (!m_pad_client->goal_responded()) return BT::NodeStatus::RUNNING;
        if (!m_pad_client->goal_accepted()) {
            RCLCPP_ERROR(m_logger, "PadRight goal rejected!");
            return BT::NodeStatus::FAILURE;
        }

        if (m_pad_client->received_result())
        {
            RCLCPP_INFO(m_logger, "PadRight result received before doing anything. Failure!");
            return BT::NodeStatus::FAILURE;
        }

        if (m_pad_client->has_right()) {
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
        if (m_pad_client) {
            m_pad_client->cancel_goal();
        }
        RCLCPP_INFO(m_logger, "GetPadRight halted, goal cancelled if it was sent.");
    }
private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;

    std::shared_ptr<PadClient> m_pad_client;
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
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client")
        };
    }

    BT::NodeStatus onStart() override
    {
        RCLCPP_INFO(m_logger, "HoldPadRight started, waiting to have the right and for Execute goal to be received...");
        if (!getInput("pad_client", m_pad_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_client]!");
            return BT::NodeStatus::FAILURE;
        }

        if (!m_pad_client->has_right() || 
            m_pad_client->received_result())
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
            if (!m_pad_client->received_result()) return BT::NodeStatus::RUNNING;
            if (m_pad_client->result_success()) {
                RCLCPP_INFO(m_logger, "Execute goal finished successfully and PadRight result success!");
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_ERROR(m_logger, "Execute goal finished cleanly, but Pad reported failure!");
                return BT::NodeStatus::FAILURE;
            } 
        }
        
        
        if (!m_pad_client->has_right() ||
            m_pad_client->received_result())
        {
            RCLCPP_ERROR(m_logger, "Cannot hold PadRight if we don't have, or if its already done!");
            return BT::NodeStatus::FAILURE;
        }

        
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        if (m_pad_client) {
            m_pad_client->cancel_goal();
        }
        RCLCPP_INFO(m_logger, "HoldPadRight halted, goal cancelled if it was sent.");
    }

private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;

    std::shared_ptr<PadClient> m_pad_client;
};

class ReleasePadRight : public BT::SyncActionNode
{
public:
    ReleasePadRight(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Logger logger)
    : BT::SyncActionNode(name, config)
    , m_logger(logger.get_child(name))
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client")
        };
    }

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(m_logger, "Releasing PadRight...");
        std::shared_ptr<PadClient> client;
        if (!getInput("pad_client", client))
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
    std::shared_ptr<PadClient> m_pad_client;
};




class LandRoutine : public BT::StatefulActionNode
{
public: 
    LandRoutine(
        const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Logger logger, 
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        std::shared_ptr<HardwareActor> hardware_actor, 
        std::shared_ptr<PadExecuteServer> pad_execute_server)
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
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client")
        };
    }

    BT::NodeStatus onStart() {
        if (!getInput("pad_client", m_pad_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_client]!");
            return BT::NodeStatus::FAILURE;
        }

        m_state = LandState::INIT;
        m_phase_start_time = m_clock->now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {
        // TODO: send feedback

        if ((m_clock->now() - m_phase_start_time) < m_phase_durations.at(m_state)) 
            return BT::NodeStatus::RUNNING;
        
        m_phase_start_time = m_clock->now();


        Eigen::Affine3d landing_pose = Eigen::Affine3d::Identity(); // TODO: get landing pose from pad client
        switch (m_state) {
            case LandState::INIT:
                RCLCPP_INFO(m_logger, "Starting land routine...");
                landing_pose = Eigen::Affine3d::Identity(); // TODO: get landing pose from pad client
                m_hardware_actor->go_to(landing_pose, 0.0, true);
                m_state = LandState::PHASE1;
                break;
            case LandState::PHASE1:
                RCLCPP_INFO(m_logger, "Phase 1: Moving to landing position...");
                landing_pose = Eigen::Affine3d::Identity(); // TODO: get descent pose from pad client
                m_hardware_actor->go_to(landing_pose, 0.0, true);
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
                return BT::NodeStatus::SUCCESS;
                break;
            default:
                RCLCPP_ERROR(m_logger, "Unknown state in LandRoutine!");
                return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() {
        RCLCPP_INFO(m_logger, "LandRoutine halted, stopping the drone. What should happen here?");
    }

private: 
    rclcpp::Logger m_logger;
    rclcpp::Clock::SharedPtr m_clock;
    std::shared_ptr<HardwareActor> m_hardware_actor;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;  
    std::shared_ptr<PadClient> m_pad_client;

    enum class LandState {
        INIT,
        PHASE1,
        PHASE2,
        PHASE3, 
        DONE 
    };
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


class ApproachIDLE : public BT::StatefulActionNode
{
public:
    ApproachIDLE(
        const std::string& name,
        const BT::NodeConfig& config,   
        rclcpp::Logger logger,
        std::shared_ptr<HardwareActor> hardware_actor,
        std::shared_ptr<PadExecuteServer> pad_execute_server)
    : BT::StatefulActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_hardware_actor(hardware_actor)
    , m_pad_execute_server(pad_execute_server)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!getInput("pad_client", m_pad_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_client]!");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(m_logger, "Approaching IDLE position...");

        Eigen::Affine3d my_pose = Eigen::Affine3d::Identity(); // TODO: get current pose from hardware actor
        Eigen::Affine3d idle_pose = Eigen::Affine3d::Identity(); // TODO: get idle pose from pad client
        bool success = m_pad_client->get_pad_idle_target(1.0, my_pose, idle_pose);
        if (!success)
        {
            RCLCPP_ERROR(m_logger, "Error getting idle target!");
            return BT::NodeStatus::FAILURE;
        }
        m_hardware_actor->go_to(idle_pose, 0.0, true);
        return BT::NodeStatus::RUNNING;
    }


    BT::NodeStatus onRunning() override
    {
        RCLCPP_INFO(m_logger, "Approaching IDLE position, waiting for completion...");
        
        // query idle target and send actor to it
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_INFO(m_logger, "ApproachIDLE halted, stopping the drone. What should happen here?");
    }

private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<HardwareActor> m_hardware_actor;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server; 
    std::shared_ptr<PadClient> m_pad_client;
};

class ApproachCLOSE : public BT::StatefulActionNode
{
public:
    ApproachCLOSE(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Logger logger,
        std::shared_ptr<HardwareActor> hardware_actor,
        std::shared_ptr<PadExecuteServer> pad_execute_server)
    : BT::StatefulActionNode(name, config)  
    , m_logger(logger.get_child(name))
    , m_hardware_actor(hardware_actor)  
    , m_pad_execute_server(pad_execute_server)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client")
        };
    }


    BT::NodeStatus onStart() override
    {
        if (!getInput("pad_client", m_pad_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_client]!");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(m_logger, "Approaching CLOSE position...");
        Eigen::Affine3d close_pose = Eigen::Affine3d::Identity(); // TODO: get close pose from pad client
        m_hardware_actor->go_to(close_pose, 0.0, true);
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        RCLCPP_INFO(m_logger, "Approaching CLOSE position, waiting for completion...");
        
        // query close target and send actor to it
        return BT::NodeStatus::RUNNING;
    }


    void onHalted() override
    {
        RCLCPP_INFO(m_logger, "ApproachCLOSE halted, stopping the drone. What should happen here?");
    }

private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<HardwareActor> m_hardware_actor;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;
    std::shared_ptr<PadClient> m_pad_client;
};


// Similar to https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/src/decorators/timeout_node.cpp
class TimeoutROS : public BT::DecoratorNode
{
public:
    TimeoutROS(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Logger logger,
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface)
        : BT::DecoratorNode(name, config)
        , m_logger(logger.get_child(name))
        , m_clock(clock_interface->get_clock())
        {
            RCLCPP_INFO(m_logger, "TimeoutROS decorator created");

        }

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("timeout_ms", 0, "Timeout in milliseconds")
            };
        }

        BT::NodeStatus tick() override
        {     
            RCLCPP_INFO(m_logger, "TimeoutROS tick, checking timeout...");
            int timeout_ms;
            if (!m_timeout_started)
            {

                if (!getInput("timeout_ms", timeout_ms))        {
                    RCLCPP_ERROR(m_logger, "Error getting input port [timeout]!");
                    return BT::NodeStatus::FAILURE;
                }

                setStatus(BT::NodeStatus::RUNNING);
                m_child_halted = false;
                m_timeout_start_time = m_clock->now();
                m_timeout_duration = rclcpp::Duration(std::chrono::milliseconds(timeout_ms));
                m_timeout_started = true;
            } else if ((m_clock->now() - m_timeout_start_time) > m_timeout_duration) {
                RCLCPP_INFO(m_logger, "Timeout reached in TimeoutROS decorator!");
                m_child_halted = true;
                haltChild();
            }



            if (m_child_halted) {
                m_timeout_started = false;
                return BT::NodeStatus::FAILURE;
            }

            const BT::NodeStatus child_status = child_node_->executeTick();
            if (isStatusCompleted(child_status)) {
                m_timeout_started = false;
                resetChild();
            }
            return child_status;
        }



        void halt() override
        {
            m_timeout_started = false;
            DecoratorNode::halt();
        }



private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<rclcpp::Clock> m_clock;
    bool m_timeout_started = false;
    rclcpp::Time m_timeout_start_time;
    rclcpp::Duration m_timeout_duration{std::chrono::milliseconds(0)};

    bool m_child_halted = false;
    
};


class SendFeedback : public BT::StatefulActionNode
{
public:
    SendFeedback(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Logger logger, 
        std::shared_ptr<PadExecuteServer> pad_execute_server)
    : BT::StatefulActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_pad_execute_server(pad_execute_server)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client")
        };
    }

    void onHalted() override
    {
        RCLCPP_INFO(m_logger, "SendFeedback halted.");
    }

    BT::NodeStatus onStart() override
    {
        std::shared_ptr<PadClient> client;
        if (!getInput("pad_client", client))
        { 
            RCLCPP_ERROR(m_logger, "Error getting input port [client]!");
            return BT::NodeStatus::FAILURE;
        }
     
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        RCLCPP_INFO(m_logger, "Sending feedback for current Execute goal...");
        //client->send_feedback();
        return BT::NodeStatus::RUNNING;
    }

private:
    rclcpp::Logger m_logger;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;
    std::shared_ptr<PadClient> m_pad_client;
};