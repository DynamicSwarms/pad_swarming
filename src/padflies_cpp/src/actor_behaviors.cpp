#include "behaviortree_cpp/bt_factory.h"
#include "Eigen/Dense"

#include "padflies_cpp/actor.hpp"
#include "rclcpp/rclcpp.hpp"

class HLCommandGoTo : public BT::SyncActionNode
{
  public: 
  HLCommandGoTo(
    const std::string& name,
    const BT::NodeConfig& config, 
    std::shared_ptr<PadflieActor> padflie_actor)
  : BT::SyncActionNode(name, config)
  , m_padflie_actor(padflie_actor)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<Eigen::Affine3d>("target"),
      BT::InputPort<double>("duration", "Duration of the movement in seconds"),
      BT::InputPort<bool>("relative", "Whether the target pose is relative to the current pose")
    };
  }

  BT::NodeStatus tick() override
  {
    if (!m_padflie_actor) {
      std::cerr << "PadflieActor not initialized!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    BT::Expected<Eigen::Affine3d> target_exp = getInput<Eigen::Affine3d>("target");
    if (!target_exp)    {
      std::cerr << "Error getting target: " << target_exp.error() << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    BT::Expected<double> duration_exp = getInput<double>("duration");
    if (!duration_exp)    {
      std::cerr << "Error getting duration: " << duration_exp.error() << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    BT::Expected<bool> relative_exp = getInput<bool>("relative");
    if (!relative_exp)    {
      std::cerr << "Error getting relative flag: " << relative_exp.error() << std::endl;
      return BT::NodeStatus::FAILURE;
    }


    Eigen::Affine3d target = target_exp.value();
    double duration = duration_exp.value();
    bool relative = relative_exp.value();
    m_padflie_actor->go_to(target, duration, relative);

    //std::cout << "Target: T:" << target.translation() << " R:" << target.rotation() << std::endl;
    return BT::NodeStatus::SUCCESS;

  }

private: 
  std::shared_ptr<PadflieActor> m_padflie_actor;
};

class HLCommandLand : public BT::SyncActionNode
{
  public: 
  HLCommandLand(
    const std::string& name,
    const BT::NodeConfig& config, 
    std::shared_ptr<PadflieActor> padflie_actor)
  : BT::SyncActionNode(name, config)
  , m_padflie_actor(padflie_actor)
  {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("height", "Height to land at in world frame"),
      BT::InputPort<double>("duration", "Duration of the landing in seconds"), 
      BT::InputPort<double>("yaw",0.0, "Yaw to maintain during landing in degrees, optional")
    };
  }

  BT::NodeStatus tick() override
  {
    if (!m_padflie_actor) {
      std::cerr << "PadflieActor not initialized!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    BT::Expected<double> height_exp = getInput<double>("height");
    if (!height_exp)    {
      std::cerr << "Error getting height: " << height_exp.error() << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    BT::Expected<double> duration_exp = getInput<double>("duration");
    if (!duration_exp)    {
      std::cerr << "Error getting duration: " << duration_exp.error() << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    BT::Expected<double> yaw_exp = getInput<double>("yaw");
    if (!yaw_exp)    {
      std::cerr << "Error getting yaw: " << yaw_exp.error() << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    std::cerr << "Landing with height: " << height_exp.value() << ", duration: " << duration_exp.value() << ", yaw: " << yaw_exp.value() << std::endl;

    double height = height_exp.value();
    double duration = duration_exp.value();
    double yaw = yaw_exp.value();
    m_padflie_actor->land(height, yaw, duration);
    return BT::NodeStatus::SUCCESS;

  }
private: 
  std::shared_ptr<PadflieActor> m_padflie_actor;
};



class CalculateAbovePadTargetAction : public BT::SyncActionNode
{
  public: 
  CalculateAbovePadTargetAction(
    const std::string& name,
    const BT::NodeConfig& config,
    std::shared_ptr<PadflieActor> padflie_actor)
  : BT::SyncActionNode(name, config)
  , m_padflie_actor(padflie_actor)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<double>("height"),
      BT::OutputPort<Eigen::Affine3d>("target") };
  }

  BT::NodeStatus tick() override
  {
    if (!m_padflie_actor) {
      std::cerr << "PadflieActor not initialized!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    BT::Expected<double> height_exp = getInput<double>("height");
    if (!height_exp)    {
      std::cerr << "Error getting height offset: " << height_exp.error() << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    double height = height_exp.value();
    std::cout << "Calculating target with height: " << height << std::endl;

    Eigen::Affine3d pad_pose;
    if (!m_padflie_actor->get_pad_pose(pad_pose)) {
      std::cerr << "Error getting pad pose!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }


    Eigen::Affine3d target_pose = Eigen::Translation3d(0.0, 0.0, height) * pad_pose;
    setOutput("target", target_pose);


    return BT::NodeStatus::SUCCESS;

  }
private:
  std::shared_ptr<PadflieActor> m_padflie_actor;
};







