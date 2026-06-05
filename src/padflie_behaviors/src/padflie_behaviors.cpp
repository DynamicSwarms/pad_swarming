#include "padflie_behaviors/padflie_behaviors.hpp"
#include <ament_index_cpp/get_package_share_path.hpp>
namespace padflie_behaviors
{

class ChoosePad : public BT::StatefulActionNode
{
public:
  ChoosePad(
    const std::string& name,
    const BT::NodeConfig& config,
    rclcpp::Logger logger, 
    std::shared_ptr<PadInfos> list_of_pad_infos,
    std::shared_ptr<PadflieTF> padflie_tf,
    std::shared_ptr<PadClientFactory> pad_client_factory)
  : BT::StatefulActionNode(name, config)
  , m_logger(logger.get_child(name))
  , m_list_of_pad_infos(list_of_pad_infos)
  , m_padflie_tf(padflie_tf)
  , m_pad_client_factory(pad_client_factory)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::OutputPort<std::shared_ptr<PadClient>>("pad_client")
    };
  }

  bool findClosestPad(std::string & closest_pad_name)
  {
    std::map<std::string, PadInfo> pad_infos;
    m_list_of_pad_infos->get_all_pad_infos(pad_infos);

    if (pad_infos.empty()) {
        RCLCPP_ERROR(m_logger, "No pads available in ChoosePad node!");
        return false;
    }
  
    auto closest_pad_it = pad_infos.end();
    double closest_distance = std::numeric_limits<double>::max();
    for (const auto& [node_name, pad_info] : pad_infos) {
      for (const std::string& tf_name : pad_info.pad_tf_names) {
        Eigen::Affine3d my_pose, pad_pose;
        if (!m_padflie_tf->get_cf_pose(my_pose)) return false;
        if (!m_padflie_tf->get_world_affine3d(tf_name, pad_pose)) return false;

        double distance = (my_pose.translation() - pad_pose.translation()).norm();
        if (distance < closest_distance) {
            closest_distance = distance;
            closest_pad_it = pad_infos.find(node_name);
        }
      }
    }

    if (closest_pad_it == pad_infos.end()) {
        RCLCPP_ERROR(m_logger, "No valid pads found in ChoosePad node!");
        return false;  // Keep running until at least one valid pad is available
    }

    closest_pad_name = closest_pad_it->second.node_name;
    RCLCPP_INFO(m_logger, "Chosen pad: %s with node name: %s", closest_pad_it->second.pad_right_control_action_name.c_str(), closest_pad_it->second.node_name.c_str());
    return true;
  }

  void onHalted() override
  {
    RCLCPP_INFO(m_logger, "ChoosePad node halted");
  }

  BT::NodeStatus onStart() override
  {
    std::string closest_pad_name;
    
    std::string current_pad_name = m_list_of_pad_infos->get_current_pad_name();
    if (!current_pad_name.empty()) {
      std::shared_ptr<PadClient> pad_client = m_pad_client_factory->create_pad_client(current_pad_name);
      setOutput("pad_client", pad_client);

      m_list_of_pad_infos->set_current_pad_name(current_pad_name); 
      return BT::NodeStatus::SUCCESS;
    } else if (findClosestPad(closest_pad_name)) {
      std::shared_ptr<PadClient> pad_client = m_pad_client_factory->create_pad_client(closest_pad_name);
      setOutput("pad_client", pad_client);
      m_list_of_pad_infos->set_current_pad_name(closest_pad_name);

      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::RUNNING; 
    }

    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    std::string closest_pad_name;
    if (findClosestPad(closest_pad_name)) {
      std::shared_ptr<PadClient> pad_client = m_pad_client_factory->create_pad_client(closest_pad_name);
      setOutput("pad_client", pad_client);
      m_list_of_pad_infos->set_current_pad_name(closest_pad_name);

      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::RUNNING; 
    }
  }

private:
  rclcpp::Logger m_logger;
  std::shared_ptr<PadInfos> m_list_of_pad_infos;
  std::shared_ptr<PadClientFactory> m_pad_client_factory;
  std::shared_ptr<PadflieTF> m_padflie_tf;
};


BT::Tree 
PadflieBehaviors::getTakeoffTree(
  BT::BehaviorTreeFactory & factory,
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  std::shared_ptr<PadExecuteServer> pad_execute_server,
  std::shared_ptr<PadClientFactory> pad_client_factory)
{
  factory.registerNodeType<ChoosePad>(
    "ChoosePad",
    m_logger,
    m_list_of_pad_infos,
    padflie_tf,
    pad_client_factory);
  std::string share_dir = ament_index_cpp::get_package_share_path("padflie_behaviors");
  std::string xml_path = share_dir + "/config/behaviors.xml";
  RCLCPP_INFO(m_logger, "Loading behavior tree XML from: %s", xml_path.c_str());
  factory.registerBehaviorTreeFromFile(xml_path);
  return factory.createTree("TakeoffBehavior");
  RCLCPP_INFO(m_logger, "Registered ChoosePad behavior in PadflieBehaviorsBase plugin and created Takeoff tree");
}


BT::Tree 
PadflieBehaviors::getLandTree(
  BT::BehaviorTreeFactory & factory,
  std::shared_ptr<HardwareActor> hardware_actor,
  std::shared_ptr<PadflieTF> padflie_tf,
  std::shared_ptr<PadExecuteServer> pad_execute_server,
  std::shared_ptr<PadClientFactory> pad_client_factory)
{
  m_list_of_pad_infos->set_current_pad_name("");
  factory.registerNodeType<ChoosePad>(
    "ChoosePad",
    m_logger,
    m_list_of_pad_infos,
    padflie_tf,
    pad_client_factory);
  std::string share_dir = ament_index_cpp::get_package_share_path("padflie_behaviors");
  std::string xml_path = share_dir + "/config/behaviors.xml";
  factory.registerBehaviorTreeFromFile(xml_path);
  return factory.createTree("LandBehavior");
  RCLCPP_INFO(m_logger, "Registered ChoosePad behavior in PadflieBehaviorsBase plugin and created Land tree");
}


}  // namespace padflie_behaviors


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(padflie_behaviors::PadflieBehaviors, padflies_cpp::IPadflieBehaviorPlugin)