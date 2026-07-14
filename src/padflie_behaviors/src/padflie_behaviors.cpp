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
    RCLCPP_INFO(m_logger, "Finding closest pad among %zu pad infos", pad_infos.size());

    bool any_available = false;
    for (const auto& [node_name, pad_info] : pad_infos) {
        if (pad_info.available) any_available = true;
    }

    if (!any_available) {
        RCLCPP_ERROR(m_logger, "No pads available in ChoosePad node!");
        return false;
    }
  
    auto closest_pad_it = pad_infos.end();
    double closest_distance = std::numeric_limits<double>::max();
    
    std::stringstream available_pads_stream;
    for (const auto& [node_name, pad_info] : pad_infos) {
        if (pad_info.available) {
            available_pads_stream << node_name << " ";
        }
    }
    RCLCPP_INFO(m_logger, "Choosing from: %s", available_pads_stream.str().c_str());
    
    for (const auto& [node_name, pad_info] : pad_infos) {
      if (!pad_info.available) continue;

      if (pad_info.charging_speed != pad_management_interfaces::msg::PadInfo::CHARGING_SPEED_FAST) {
          RCLCPP_INFO(m_logger, "Skipping pad %s because it is not fast charging.", node_name.c_str());
          continue;
      }
      

      for (const std::string& tf_name : pad_info.pad_tf_names) {
        Eigen::Affine3d my_pose, pad_pose;
        if (!m_padflie_tf->get_cf_pose(my_pose)) return false;
        if (!m_padflie_tf->can_transform_world(tf_name)) continue;
        if (!m_padflie_tf->get_world_affine3d(tf_name, pad_pose)) continue;

        RCLCPP_DEBUG(m_logger, "Pad %s has TF %s with pose translation: [%f, %f, %f], and we are at pose translation: [%f, %f, %f]", 
                    node_name.c_str(), 
                    tf_name.c_str(),
                    pad_pose.translation().x(), pad_pose.translation().y(), pad_pose.translation().z(),
                    my_pose.translation().x(), my_pose.translation().y(), my_pose.translation().z());
                    

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
      RCLCPP_INFO(m_logger, "Current pad already set to %s, using it.", current_pad_name.c_str());
      std::shared_ptr<PadClient> pad_client = m_pad_client_factory->create_pad_client(current_pad_name);
      setOutput("pad_client", pad_client);
      return BT::NodeStatus::SUCCESS;
    } else if (findClosestPad(closest_pad_name)) {
      std::shared_ptr<PadClient> pad_client = m_pad_client_factory->create_pad_client(closest_pad_name);
      setOutput("pad_client", pad_client);
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



class ReleasePadRight : public BT::SyncActionNode
{
public:
    ReleasePadRight(
        const std::string& name,
        const BT::NodeConfig& config,
        rclcpp::Logger logger, 
        std::shared_ptr<PadExecuteServer> server,
        std::shared_ptr<PadInfos> list_of_pad_infos)
    : BT::SyncActionNode(name, config)
    , m_logger(logger.get_child(name))
    , m_pad_execute_server(server)
    , m_list_of_pad_infos(list_of_pad_infos)
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
            m_list_of_pad_infos->set_current_pad_name(client->get_pad_name()); 
        } else if (status == pad_management_interfaces::action::PadExecute::Feedback::STATUS_TAKEOFF_CLEARED_PAD) {
            m_pad_execute_server->send_result(pad_management_interfaces::action::PadExecute::Result::RESULT_NOT_ON_PAD);
            m_list_of_pad_infos->set_current_pad_name(""); 

        } else {
            m_pad_execute_server->send_result(pad_management_interfaces::action::PadExecute::Result::RESULT_FAILURE);
            m_list_of_pad_infos->set_current_pad_name(""); 

        }

        return BT::NodeStatus::SUCCESS;
    }

private: 
    rclcpp::Logger m_logger;
    std::shared_ptr<PadExecuteServer> m_pad_execute_server;
    std::shared_ptr<PadInfos> m_list_of_pad_infos;
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
  factory.registerNodeType<ReleasePadRight>("ReleasePadRight", m_logger, pad_execute_server, m_list_of_pad_infos);


  std::string share_dir = ament_index_cpp::get_package_share_path("padflie_behaviors");
  std::string xml_path = share_dir + "/config/behaviors.xml";
  RCLCPP_INFO(m_logger, "Loading behavior tree XML from: %s", xml_path.c_str());
  factory.registerBehaviorTreeFromFile(xml_path);
  return factory.createTree("TakeoffBehavior");
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
  factory.registerNodeType<ReleasePadRight>("ReleasePadRight", m_logger, pad_execute_server, m_list_of_pad_infos);

  std::string share_dir = ament_index_cpp::get_package_share_path("padflie_behaviors");
  std::string xml_path = share_dir + "/config/behaviors.xml";
  factory.registerBehaviorTreeFromFile(xml_path);
  return factory.createTree("LandBehavior");
}


}  // namespace padflie_behaviors


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(padflie_behaviors::PadflieBehaviors, padflies_cpp::IPadflieBehaviorPlugin)