#pragma once
#include "behaviortree_cpp/bt_factory.h"

#include "padflies_cpp/pad_execute_server.hpp"
#include "padflies_cpp/pad_client.hpp"
#include "padflies_cpp/pad_client_factory.hpp"
#include "Eigen/Dense"
#include "padflies_cpp/hardware_actor.hpp"

#include <tf2/utils.hpp>

using namespace std::chrono_literals;

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
        return BT::NodeStatus::RUNNING;
    }



    BT::NodeStatus onRunning() override
    {
        RCLCPP_DEBUG(m_logger, "GetPadRight running, waiting to acquire right and for Execute goal...");
        if (!m_request_sent)
        {
            if (m_pad_client->is_action_server_available()) {
                RCLCPP_INFO(m_logger, "PadRight action server is available, sending goal...");
                m_pad_client->send_request(m_action);
                m_request_sent = true;
            }   
        }
    
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
            RCLCPP_DEBUG(m_logger, "Still waiting to acquire PadRight...");
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        if (m_pad_client && !m_pad_client->is_action_server_available()){
            RCLCPP_WARN(m_logger, "Halted but PadRight action server is not available, maybe that is the reason");
        }
        if (m_pad_client && m_request_sent) {
            m_pad_client->cancel_goal();
        }
        RCLCPP_INFO(m_logger, "GetPadRight halted, goal cancelled if it was sent.");
    }
private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;

    std::shared_ptr<PadClient> m_pad_client;
    uint8_t m_action;

    bool m_request_sent = false;
};

class HasPadRight : public BT::ConditionNode
{
public: 
    HasPadRight(
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
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client")
        };
    }

    BT::NodeStatus tick() override
    {
        std::shared_ptr<PadClient> client;
        if (!getInput("pad_client", client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_client]!");
            return BT::NodeStatus::FAILURE;
        }

        if (client->has_right()) {
            RCLCPP_DEBUG(m_logger, "Condition HasPadRight SUCCESS");
            return BT::NodeStatus::SUCCESS;
        } else {
            RCLCPP_INFO(m_logger, "Condition HasPadRight FAILURE");
            return BT::NodeStatus::FAILURE;
        }
    }
private:
    rclcpp::Logger m_logger;
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
        rclcpp::Logger logger, 
        std::shared_ptr<PadExecuteServer> server)
    : BT::SyncActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_pad_execute_server(server)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client"), 
            BT::InputPort<uint8_t>("status")
        };
    }

    BT::NodeStatus tick() override
    {
        RCLCPP_INFO(m_logger, "Releasing PadRight...");
        std::shared_ptr<PadClient> client;
        uint8_t status;
         
        if (!getInput("pad_client", client))
        { 
            RCLCPP_ERROR(m_logger, "Error getting input port [client]!");
            return BT::NodeStatus::FAILURE;
        }
     
        if (!getInput("status", status))
        { 
            RCLCPP_ERROR(m_logger, "Error getting input port [status]!");
            return BT::NodeStatus::FAILURE;
        }

        if (status == pad_management_interfaces::action::PadExecute::Feedback::STATUS_LANDED)
        {
            m_pad_execute_server->send_result(pad_management_interfaces::action::PadExecute::Result::RESULT_ON_PAD);
        } else if (status == pad_management_interfaces::action::PadExecute::Feedback::STATUS_TAKEOFF_CLEARED_PAD) {
            m_pad_execute_server->send_result(pad_management_interfaces::action::PadExecute::Result::RESULT_NOT_ON_PAD);
        } else {
            m_pad_execute_server->send_result(pad_management_interfaces::action::PadExecute::Result::RESULT_FAILURE);
        }

        return BT::NodeStatus::SUCCESS;
    }

private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;
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
        std::shared_ptr<PadflieTF> padflie_tf,
        std::shared_ptr<PadExecuteServer> pad_execute_server)
    : BT::StatefulActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_clock(node_clock_interface->get_clock())
    , m_hardware_actor(hardware_actor)
    , m_padflie_tf(padflie_tf)
    , m_pad_execute_server(pad_execute_server)
    {
        m_state = LandState::INIT;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client"), 
            BT::OutputPort<uint8_t>("status")
        };
    }

    bool get_target_world_frame(Eigen::Affine3d& target_world_frame)
    {
        geometry_msgs::msg::PoseStamped target_pose = m_pad_client->get_target_pose();
        Eigen::Affine3d target_remote_frame;
        
        tf2::fromMsg(target_pose.pose, target_remote_frame);
        if (m_padflie_tf->affine3d_transform(
            target_remote_frame, 
            target_pose.header.frame_id, 
            "world", 
            target_world_frame))
        {
            return true;
        } else {
            RCLCPP_ERROR(m_logger, "Failed to transform target pose from frame [%s] to world frame!", target_pose.header.frame_id.c_str());
            return false;
        }
    }

    BT::NodeStatus onStart() {
        if (!getInput("pad_client", m_pad_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_client]!");
            return BT::NodeStatus::FAILURE;
        }


        // If we are close to the target, we can make PHASE1 shorter.
        Eigen::Affine3d target_world_frame;
        if (!get_target_world_frame(target_world_frame))        {
            RCLCPP_ERROR(m_logger, "Failed to get target pose in world frame!");
            return BT::NodeStatus::FAILURE;
        }
        Eigen::Vector3d position; 
        if (m_padflie_tf->get_cf_position(position))
        {
            if ((position - target_world_frame.translation()).norm() < 1.0) 
            {
                RCLCPP_INFO(m_logger, "are close to the target position!");
                m_phase_durations.at(LandState::PHASE1) = rclcpp::Duration(1250ms);
            }
        } else 
        {
            RCLCPP_ERROR(m_logger, "Error getting Crazyflie position!");
            return BT::NodeStatus::FAILURE;
        }


        m_state = LandState::INIT;
        setOutput("status", pad_management_interfaces::action::PadExecute::Feedback::STATUS_LANDING);
        m_phase_start_time = m_clock->now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {        
        //RCLCPP_INFO(m_logger, "LandRoutine state machine tick, current state: %d", static_cast<int>(m_state));
        //RCLCPP_INFO(m_logger, "Time since phase start: %f seconds, phase_duration: %f", (m_clock->now() - m_phase_start_time).seconds(), m_phase_durations.at(m_state).seconds());
        if ((m_clock->now() - m_phase_start_time) < m_phase_durations.at(m_state)) 
            return BT::NodeStatus::RUNNING;        
        m_phase_start_time = m_clock->now();

        
        Eigen::Affine3d target_world_frame;
        if (!get_target_world_frame(target_world_frame))
        {
            RCLCPP_ERROR(m_logger, "Failed to get target pose in world frame!");
            return BT::NodeStatus::FAILURE;
        }


        //RCLCPP_INFO(m_logger, "Target pose in world frame: [%f, %f, %f]", target_world_frame.translation().x(), target_world_frame.translation().y(), target_world_frame.translation().z());
        switch (m_state) {
            case LandState::INIT:
                RCLCPP_INFO(m_logger, "Starting land routine...");
                m_hardware_actor->go_to(
                    target_world_frame * Eigen::Translation3d(0, 0, 0.25),
                    m_phase_durations.at(LandState::PHASE1).seconds(),
                    false);
                m_state = LandState::PHASE1;
                break;
            case LandState::PHASE1:
                RCLCPP_INFO(m_logger, "Phase 1: Moving to landing position...");
                m_hardware_actor->go_to(
                    target_world_frame * Eigen::Translation3d(0, 0, -0.1),
                    3.0,
                    false);
                m_state = LandState::PHASE2;
                break;
            case LandState::PHASE2:
                RCLCPP_INFO(m_logger, "Phase 2: Final descent...");

                m_hardware_actor->land(
                    (target_world_frame * Eigen::Translation3d(0, 0, -0.5)).translation().z(),
                    std::atan2(target_world_frame.rotation()(1,0), target_world_frame.rotation()(0,0)),
                    3.0);
                m_state = LandState::DONE;
                break;
            case LandState::DONE:
                // Send out PadExecute Result with success.

                setOutput("status", pad_management_interfaces::action::PadExecute::Feedback::STATUS_LANDED);
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
    std::shared_ptr<PadflieTF> m_padflie_tf;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;  
    std::shared_ptr<PadClient> m_pad_client;

    enum class LandState {
        INIT,
        PHASE1,
        PHASE2,
        DONE 
    };
    LandState m_state = LandState::INIT;
    std::map<LandState, rclcpp::Duration> m_phase_durations = {
        {LandState::INIT, rclcpp::Duration(0s)},
        {LandState::PHASE1, rclcpp::Duration(4500ms)},
        {LandState::PHASE2, rclcpp::Duration(1000ms)},
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
        std::shared_ptr<PadflieTF> padflie_tf,
        std::shared_ptr<PadExecuteServer> pad_execute_server)
    : BT::StatefulActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_hardware_actor(hardware_actor)
    , m_padflie_tf(padflie_tf)
    , m_pad_execute_server(pad_execute_server)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client"), 
            BT::OutputPort<uint8_t>("status")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!getInput("pad_client", m_pad_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_client]!");
            return BT::NodeStatus::FAILURE;
        }
        setOutput("status", pad_management_interfaces::action::PadExecute::Feedback::STATUS_LANDING_APPROACH_IDLE);
        RCLCPP_INFO(m_logger, "Approaching IDLE position...");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        RCLCPP_INFO(m_logger, "Approaching IDLE position...");

        Eigen::Affine3d my_pose;
        if (!m_padflie_tf->get_cf_pose(my_pose))
        {
            RCLCPP_ERROR(m_logger, "Error getting Crazyflie position!");
            return BT::NodeStatus::FAILURE;
        } else {
            RCLCPP_DEBUG(m_logger, "Current Crazyflie position: [%f, %f, %f]", my_pose.translation().x(), my_pose.translation().y(), my_pose.translation().z());
        }
        Eigen::Affine3d idle_pose_remote_frame;
        std::string target_frame_id;
        bool success = m_pad_client->get_pad_idle_target(1.0, my_pose, "world", idle_pose_remote_frame, target_frame_id);
        if (!success)
        {
            RCLCPP_ERROR(m_logger, "Error getting idle target!");
            return BT::NodeStatus::FAILURE;
        }
  

        PoseTarget idle_pose_target;
        idle_pose_target.frame_id = target_frame_id;
        idle_pose_target.pose = idle_pose_remote_frame;
        idle_pose_target.use_yaw = true;
        idle_pose_target.collision_avoidance = true;

        m_hardware_actor->set_pose_target(idle_pose_target);
        return BT::NodeStatus::RUNNING;
    }


    void onHalted() override
    {
        RCLCPP_INFO(m_logger, "ApproachIDLE halted, stopping the drone. What should happen here?");
    }

private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<HardwareActor> m_hardware_actor;
    std::shared_ptr<PadflieTF> m_padflie_tf;
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
        std::shared_ptr<PadflieTF> padflie_tf,
        std::shared_ptr<PadExecuteServer> pad_execute_server)
    : BT::StatefulActionNode(name, config)  
    , m_logger(logger.get_child(name))
    , m_hardware_actor(hardware_actor)  
    , m_padflie_tf(padflie_tf)
    , m_pad_execute_server(pad_execute_server)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client"),
            BT::OutputPort<uint8_t>("status")
        };
    }


    BT::NodeStatus onStart() override
    {
        if (!getInput("pad_client", m_pad_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_client]!");
            return BT::NodeStatus::FAILURE;
        }

        setOutput("status", pad_management_interfaces::action::PadExecute::Feedback::STATUS_LANDING_APPROACH_CLOSE);
        RCLCPP_INFO(m_logger, "Approaching CLOSE position...");
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        RCLCPP_DEBUG(m_logger, "Approaching CLOSE position, waiting for completion...");
        
        geometry_msgs::msg::PoseStamped close_target_pose = m_pad_client->get_target_pose();
        close_target_pose.pose.position.z += 0.5; // hover 0.5m above the target pose
        
        PoseTarget close_target;
        close_target.frame_id = close_target_pose.header.frame_id;
        tf2::fromMsg(close_target_pose.pose, close_target.pose);
        close_target.use_yaw = true;
        close_target.collision_avoidance = true;
        m_hardware_actor->set_pose_target(close_target);

        Eigen::Vector3d position; 
        if (m_padflie_tf->get_cf_position(position))
        {
            geometry_msgs::msg::PoseStamped close_target_world_frame;
            if (m_padflie_tf->transform_pose_stamped(close_target_pose, "world", close_target_world_frame))
            {
                Eigen::Affine3d close_target_world_affine = Eigen::Affine3d::Identity();
                tf2::fromMsg(close_target_world_frame.pose, close_target_world_affine);

                if ((position - close_target_world_affine.translation()).norm() < 0.5) {
                    RCLCPP_DEBUG(m_logger, "Reached CLOSE position!");
                    return BT::NodeStatus::SUCCESS;
                }
            } else {
                RCLCPP_ERROR(m_logger, "Error transforming CLOSE target pose to world frame!");
                return BT::NodeStatus::FAILURE;
            }
        } else {
            RCLCPP_ERROR(m_logger, "Error getting Crazyflie position!");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }


    void onHalted() override
    {
        RCLCPP_INFO(m_logger, "ApproachCLOSE halted, stopping the drone. What should happen here?");
    }

private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<HardwareActor> m_hardware_actor;
    std::shared_ptr<PadflieTF> m_padflie_tf;
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
        {}

        static BT::PortsList providedPorts()
        {
            return {
                BT::InputPort<int>("timeout_ms", 0, "Timeout in milliseconds")
            };
        }

        BT::NodeStatus tick() override
        {     
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


class SendFeedback : public BT::SyncActionNode
{
public:
    SendFeedback(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Logger logger, 
        std::shared_ptr<PadExecuteServer> pad_execute_server)
    : BT::SyncActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_pad_execute_server(pad_execute_server)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client"), 
            BT::InputPort<uint8_t>("status")
        };
    }

    BT::NodeStatus tick() override
    {
        std::shared_ptr<PadClient> client;
        if (!getInput("pad_client", client))
        { 
            RCLCPP_ERROR(m_logger, "Error getting input port [client]!");
            return BT::NodeStatus::FAILURE;
        }
        uint8_t status;
        if (!getInput("status", status))
        { 
            RCLCPP_ERROR(m_logger, "Error getting input port [status]!");
            return BT::NodeStatus::FAILURE;
        }
        m_pad_execute_server->send_feedback(status);

        
        RCLCPP_DEBUG(m_logger, "Sending feedback from SendFeedback node...");
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Logger m_logger;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;
    std::shared_ptr<PadClient> m_pad_client;
};


class TakeoffRoutine : public BT::StatefulActionNode
{
public: 
    TakeoffRoutine(
        const std::string& name, 
        const BT::NodeConfig& config,
        rclcpp::Logger logger, 
        std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> node_clock_interface,
        std::shared_ptr<HardwareActor> hardware_actor, 
        std::shared_ptr<PadflieTF> padflie_tf,
        std::shared_ptr<PadExecuteServer> pad_execute_server)
    : BT::StatefulActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_clock(node_clock_interface->get_clock())
    , m_hardware_actor(hardware_actor)
    , m_padflie_tf(padflie_tf)
    , m_pad_execute_server(pad_execute_server)
    {
        m_state = TakeoffState::INIT;
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<PadClient>>("pad_client"), 
            BT::OutputPort<uint8_t>("status")
        };
    }

    bool get_target_world_frame(Eigen::Affine3d& target_world_frame)
    {
        geometry_msgs::msg::PoseStamped target_pose = m_pad_client->get_target_pose();
        Eigen::Affine3d target_remote_frame;
        
        
        tf2::fromMsg(target_pose.pose, target_remote_frame);
        if (m_padflie_tf->affine3d_transform(
            target_remote_frame, 
            target_pose.header.frame_id, 
            "world", 
            target_world_frame))
        {
            return true;
        } else {
            RCLCPP_ERROR(m_logger, "Failed to transform target pose from frame [%s] to world frame!", target_pose.header.frame_id.c_str());
            return false;
        }
    }

    BT::NodeStatus onStart() {
        if (!getInput("pad_client", m_pad_client))
        {
            RCLCPP_ERROR(m_logger, "Error getting input port [pad_client]!");
            return BT::NodeStatus::FAILURE;
        }

        Eigen::Vector3d position;
        if (m_padflie_tf->get_cf_position(position))
        {
            RCLCPP_INFO(m_logger, "Current Crazyflie position: [%f, %f, %f]", position.x(), position.y(), position.z());
        } else {
            RCLCPP_ERROR(m_logger, "Error getting Crazyflie position!");
            return BT::NodeStatus::FAILURE;
        }


        m_state = TakeoffState::INIT;
        setOutput("status", pad_management_interfaces::action::PadExecute::Feedback::STATUS_TAKEOFF_IN_PAD);
        m_phase_start_time = m_clock->now();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() {        
        //RCLCPP_INFO(m_logger, "LandRoutine state machine tick, current state: %d", static_cast<int>(m_state));
        //RCLCPP_INFO(m_logger, "Time since phase start: %f seconds, phase_duration: %f", (m_clock->now() - m_phase_start_time).seconds(), m_phase_durations.at(m_state).seconds());
        if ((m_clock->now() - m_phase_start_time) < m_phase_durations.at(m_state)) 
            return BT::NodeStatus::RUNNING;        
        m_phase_start_time = m_clock->now();

        
        Eigen::Affine3d target_world_frame;
        if (!get_target_world_frame(target_world_frame))
        {
            RCLCPP_ERROR(m_logger, "Failed to get target pose in world frame!");
            return BT::NodeStatus::FAILURE;
        }

        switch (m_state) {
            case TakeoffState::INIT:
                RCLCPP_INFO(m_logger, "Starting takeoff routine... resetting Kalman filter...");
                m_hardware_actor->reset_kalman_to(target_world_frame);

                m_state = TakeoffState::PHASE1;
                break;
            case TakeoffState::PHASE1:
                RCLCPP_INFO(m_logger, "Starting takeoff routine... moving up to clear the pad...");
                m_hardware_actor->go_to(
                    Eigen::Affine3d::Identity() * Eigen::Translation3d(0,0, 0.1),
                    3.0,
                    true); // relative move up by 0.1m from current position
                m_state = TakeoffState::PHASE2;
                break;
            case TakeoffState::PHASE2:
                RCLCPP_INFO(m_logger, "Phase 2: Moving higher...");
                m_hardware_actor->go_to(
                    Eigen::Affine3d::Identity() * Eigen::Translation3d(0,0, 0.6),
                    1.5,
                    true); // relative move up by 0.1m from current position
                setOutput("status", pad_management_interfaces::action::PadExecute::Feedback::STATUS_TAKEOFF_LEFT_PAD);
                m_state = TakeoffState::PHASE3;
                break;
            case TakeoffState::PHASE3:
                RCLCPP_INFO(m_logger, "Phase 3: Final ascent...");
                {
                    PoseTarget takeoff_target;
                    takeoff_target.frame_id = "world";
                    takeoff_target.pose = target_world_frame * Eigen::Translation3d(0, 0, 1.0); // move to a point above the target pose
                    takeoff_target.use_yaw = true;
                    takeoff_target.collision_avoidance = true;

                    bool success = m_hardware_actor->set_pose_target(takeoff_target);
                    if (!success) {
                        RCLCPP_ERROR(m_logger, "Failed to set takeoff target pose!");
                        return BT::NodeStatus::FAILURE;
                    }
                }

                m_state = TakeoffState::DONE;
                break;
            case TakeoffState::DONE:
                // Send out PadExecute Result with success.

                setOutput("status", pad_management_interfaces::action::PadExecute::Feedback::STATUS_TAKEOFF_CLEARED_PAD);
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
    std::shared_ptr<PadflieTF> m_padflie_tf;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;  
    std::shared_ptr<PadClient> m_pad_client;

    enum class TakeoffState {
        INIT,
        PHASE1,
        PHASE2,
        PHASE3,
        DONE 
    };
    TakeoffState m_state = TakeoffState::INIT;
    std::map<TakeoffState, rclcpp::Duration> m_phase_durations = {
        {TakeoffState::INIT, rclcpp::Duration(0s)},
        {TakeoffState::PHASE1, rclcpp::Duration(250ms)},
        {TakeoffState::PHASE2, rclcpp::Duration(250ms)},
        {TakeoffState::PHASE3, rclcpp::Duration(250ms)},
        {TakeoffState::DONE, rclcpp::Duration(1250ms)}
    };
    rclcpp::Time m_phase_start_time;
};


class TakeoffInit : public BT::SyncActionNode
{
public:
    TakeoffInit(
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
            BT::OutputPort<uint8_t>("status")
        };
    }

    BT::NodeStatus tick() override
    {
        setOutput("status", pad_management_interfaces::action::PadExecute::Feedback::STATUS_TAKEOFF_INIT);
        RCLCPP_INFO(m_logger, "TakeoffInit ticked, setting status to TAKEOFF_IN_PAD");
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Logger m_logger;
};

class LandInit : public BT::SyncActionNode
{
public:
    LandInit(
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
            BT::OutputPort<uint8_t>("status")
        };
    }

    BT::NodeStatus tick() override
    {
        setOutput("status", pad_management_interfaces::action::PadExecute::Feedback::STATUS_LANDING_INIT);
        RCLCPP_INFO(m_logger, "LandInit ticked, setting status to LAND_INIT");
        return BT::NodeStatus::SUCCESS;
    }
private:
    rclcpp::Logger m_logger;
};

class TryFinally : public BT::ControlNode
{
public:
    TryFinally(
        const std::string& name,
        const BT::NodeConfig& config, 
        rclcpp::Logger logger)
    : BT::ControlNode(name, config)
    , m_logger(logger.get_child(name))
    {
    }

    ~TryFinally() override = default;

    static BT::PortsList providedPorts()
    {
        return {};
    }
    
    BT::NodeStatus tick() override
    {
        if (children_nodes_.size() != 2) {
            RCLCPP_ERROR(m_logger, "TryFinally node must have exactly 2 children!");
            return BT::NodeStatus::FAILURE;
        }

        if (!m_finally_started) 
        {

            // Execute the first child (the "try" block)
            m_try_status = children_nodes_[0]->executeTick();

            // If the "try" block is still running, keep it running
            if (m_try_status == BT::NodeStatus::RUNNING) {
                return BT::NodeStatus::RUNNING;
            } else if ( m_try_status == BT::NodeStatus::FAILURE) {
                RCLCPP_ERROR(m_logger, "Try block failed -> doing finally block");
                m_finally_started = true;
            } else if (m_try_status == BT::NodeStatus::SUCCESS) {
                RCLCPP_INFO(m_logger, "Try block succeeded -> doing finally block");
                m_finally_started = true;
            }
        }

        // Once the "try" block is done (either SUCCESS or FAILURE), execute the second child (the "finally" block)
        BT::NodeStatus finally_status = children_nodes_[1]->executeTick();

        // The status of the TryFinally node is determined by the "try" block, but we need to make sure the "finally" block runs to completion
        if (finally_status == BT::NodeStatus::RUNNING) {
            return BT::NodeStatus::RUNNING;
        }

        // If we reach here, both blocks have finished. The overall status is determined by the "try" block.
        return m_try_status;
    }

    void halt() override
    {
        // Halt both children 
        for (size_t i = 0; i < children_nodes_.size(); ++i) {
            haltChild(i);
        }
    }

private: 
    rclcpp::Logger m_logger;
    bool m_finally_started = false;
    BT::NodeStatus m_try_status = BT::NodeStatus::IDLE;

};