

class SplitPose : public BT::SyncActionNode
{
  public: 
  SplitPose(
    const std::string& name,
    const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<Eigen::Affine3d>("pose"),
      BT::OutputPort<double>("x"),
      BT::OutputPort<double>("y"),
      BT::OutputPort<double>("z"),
      BT::OutputPort<double>("yaw") };
  }

  BT::NodeStatus tick() override
  {
    Eigen::Affine3d pose;
    if (!getInput<Eigen::Affine3d>("pose", pose))    {
      std::cerr << "Error getting pose input: " << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    
    setOutput("x", pose.translation().x());
    setOutput("y", pose.translation().y());
    setOutput("z", pose.translation().z());
    setOutput("yaw", std::atan2(pose.rotation().col(0).y(), pose.rotation().col(0).x()) * 180.0 / M_PI);
    return BT::NodeStatus::SUCCESS;
  }
};

class ExtractYawDeg : public BT::SyncActionNode
{
  public: 
  ExtractYawDeg(
    const std::string& name,
    const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<Eigen::Affine3d>("pose"),
      BT::OutputPort<double>("yaw") };
  }

  BT::NodeStatus tick() override
  {
    BT::Expected<Eigen::Affine3d> pose_exp = getInput<Eigen::Affine3d>("pose");
    if (!pose_exp)    {
      std::cerr << "Error getting pose: " << pose_exp.error() << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    Eigen::Affine3d pose = pose_exp.value();

    double yaw = std::atan2(pose.rotation().col(0).y(), pose.rotation().col(0).x()) * 180.0 / M_PI;

    setOutput("yaw", yaw);
    return BT::NodeStatus::SUCCESS;
  }
};

class ExtractHeight : public BT::SyncActionNode
{
  public: 
  ExtractHeight(
    const std::string& name,
    const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts()
  {
    return { 
      BT::InputPort<Eigen::Affine3d>("pose"),
      BT::OutputPort<double>("height") };
  }

  BT::NodeStatus tick() override
  {
    BT::Expected<Eigen::Affine3d> pose_exp = getInput<Eigen::Affine3d>("pose");
    if (!pose_exp)    {
      std::cerr << "Error getting pose: " << pose_exp.error() << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    Eigen::Affine3d pose = pose_exp.value();

    double height = pose.translation().z();

    setOutput("height", height);
    return BT::NodeStatus::SUCCESS;
  }
};