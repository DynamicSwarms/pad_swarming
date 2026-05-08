#include "behaviortree_cpp/bt_factory.h"
#include "Eigen/Dense"

#include "padflies_cpp/hardware_actor.hpp"
#include "rclcpp/rclcpp.hpp"

class HLCommandGoTo : public BT::SyncActionNode
{
  public: 
  HLCommandGoTo(
    const std::string& name,
    const BT::NodeConfig& config, 
    std::shared_ptr<HardwareActor> actor, 
    rclcpp::Logger logger)
  : BT::SyncActionNode(name, config)
  , m_actor(actor)
, m_logger(logger.get_child(name))
  {
    if (!m_actor) {
      throw std::runtime_error("HardwareActor not initialized");
    }
  }

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<Eigen::Affine3d>("target", "Target pose to go to in world frame (or relative)"),
      BT::InputPort<double>("duration", "Duration of the movement in seconds"),
      BT::InputPort<bool>("relative", "Whether the target pose is relative to the current pose")
    };
  }

  BT::NodeStatus tick() override
  {
    Eigen::Affine3d target;
    double duration;
    bool relative;
    
    if (!getInput("target", target) || !getInput("duration", duration) || !getInput("relative", relative)) {
      RCLCPP_ERROR(m_logger, "Error getting input ports!");
      return BT::NodeStatus::FAILURE;
    }


    if (m_actor->go_to(target, duration, relative)) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(m_logger, "Error executing go_to command!");
      return BT::NodeStatus::FAILURE;
    }

  }

private: 
  std::shared_ptr<HardwareActor> m_actor;
  rclcpp::Logger m_logger;
};

class HLCommandLand : public BT::SyncActionNode
{
  public: 
  HLCommandLand(
    const std::string& name,
    const BT::NodeConfig& config, 
    std::shared_ptr<HardwareActor> actor,
    rclcpp::Logger logger)
  : BT::SyncActionNode(name, config)
  , m_actor(actor)
  , m_logger(logger.get_child(name))
  {
    if (!m_actor) {
      throw std::runtime_error("HardwareActor not initialized");
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("height", 0.0, "Height to land at in world frame"),
      BT::InputPort<double>("duration", 4.0, "Duration of the landing in seconds"), 
      BT::InputPort<double>("yaw",0.0, "Yaw to maintain during landing in degrees, optional")
    };
  }

  BT::NodeStatus tick() override
  {
    double height;
    double duration;
    double yaw;
    if (!getInput("height", height) || !getInput("duration", duration) || !getInput("yaw", yaw)) 
    {
      RCLCPP_ERROR(m_logger, "Error getting input ports!");
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(m_logger, "Landing with height: %f, duration: %f, yaw: %f", height, duration, yaw);
    if (m_actor->land(height, yaw, duration)) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(m_logger, "Error executing land command!");
      return BT::NodeStatus::FAILURE;
    }

  }
private: 
  std::shared_ptr<HardwareActor> m_actor;
  rclcpp::Logger m_logger;
};

class HLCommandTakeoff : public BT::SyncActionNode
{
  public: 
  HLCommandTakeoff(
    const std::string& name,
    const BT::NodeConfig& config, 
    std::shared_ptr<HardwareActor> actor,
    rclcpp::Logger logger)
  : BT::SyncActionNode(name, config)
  , m_actor(actor)
  , m_logger(logger.get_child(name))
  {
    if (!m_actor) {
      throw std::runtime_error("HardwareActor not initialized");
    }
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("height", 1.0,  "Height to takeoff to in world frame"),
      BT::InputPort<double>("duration", 4.0,  "Duration of the takeoff in seconds"), 
      BT::InputPort<double>("yaw",0.0, "Yaw to maintain during takeoff in degrees, optional")
    };
  }

  BT::NodeStatus tick() override
  {
    double height;
    double duration;
    double yaw;
    if (!getInput("height", height) || !getInput("duration", duration) || !getInput("yaw", yaw)) 
    {
      RCLCPP_ERROR(m_logger, "Error getting input ports!");
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_INFO(m_logger, "Taking off with height: %f, duration: %f, yaw: %f", height, duration, yaw);

    if (m_actor->takeoff(height, yaw, duration)) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(m_logger, "Error executing takeoff command!");
      return BT::NodeStatus::FAILURE;
    }
  }

private: 
  std::shared_ptr<HardwareActor> m_actor;
  rclcpp::Logger m_logger;
};

class LLCommanderSendTarget : public BT::SyncActionNode
{
  public: 
  LLCommanderSendTarget(
    const std::string& name,
    const BT::NodeConfig& config, 
    std::shared_ptr<HardwareActor> actor, 
    rclcpp::Logger logger
  )
  : BT::SyncActionNode(name, config)
  , m_actor(actor)
  , m_logger(logger.get_child(name))
  {
    if (!m_actor) {
      throw std::runtime_error("HardwareActor not initialized");
    }
  }

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<Eigen::Affine3d>("target_pose", "Target pose to send to low-level commander"),
      BT::InputPort<std::string>("target_frame", "Frame of the target pose"),
      BT::InputPort<bool>("use_yaw", true, "Whether to use yaw from the target pose") };
  }

  BT::NodeStatus tick() override
  {
    Eigen::Affine3d target_pose;
    std::string target_frame;
    bool use_yaw;
    if (!getInput("target_pose", target_pose) || !getInput("target_frame", target_frame) || !getInput("use_yaw", use_yaw)) 
    {
      RCLCPP_ERROR(m_logger, "Error getting input ports!");
      return BT::NodeStatus::FAILURE;
    }

    EigenPoseStamped target_pose_stamped;

    target_pose_stamped.pose = target_pose;

    target_pose_stamped.frame_id = target_frame;

    if (m_actor->set_pose_target(target_pose_stamped, use_yaw)) {
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(m_logger, "Error sending target to low-level commander!");
      return BT::NodeStatus::FAILURE;
    }
  }

private: 
  std::shared_ptr<HardwareActor> m_actor;
  rclcpp::Logger m_logger;
};







