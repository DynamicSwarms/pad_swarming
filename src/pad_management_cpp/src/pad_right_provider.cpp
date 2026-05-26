#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <string>
#include <memory>

#include "pluginlib/class_loader.hpp"

#include "pad_management_cpp/pad_right_server.hpp"

class PadRightActionServerNode : public rclcpp::Node
{
public:
  PadRightActionServerNode()
  : Node("megapad")
  , m_pad_resource_manager_loader("pad_management_cpp", "IPadResourceManager")
  , m_pad_resource_manager(create_pad_resource_manager())
  , m_pad_right_server(std::make_shared<PadRightServer>(
      this->get_node_base_interface()->get_name(),
      *m_pad_resource_manager,
      this->get_node_base_interface(),
      this->get_node_parameters_interface(),
      this->get_node_timers_interface(),
      this->get_node_topics_interface(),
      this->get_node_graph_interface(),
      this->get_node_clock_interface(),
      this->get_node_waitables_interface(),
      this->get_node_logging_interface()
    ))
  {
    RCLCPP_INFO(this->get_logger(), "PadRightActionServerNode has been initialized.");
  }

private:
  std::shared_ptr<IPadResourceManager> 
  create_pad_resource_manager()
  {
    const auto plugin_name = this->declare_parameter<std::string>(
      "pad_resource_manager_plugin",
      "pad_management_cpp::PadResourceManager");
    RCLCPP_INFO(this->get_logger(), "Loading PadResourceManager plugin: %s", plugin_name.c_str());

    pad_management_cpp::NodeInterfacesBundle node_interfaces_bundle{
        this->get_node_base_interface(),
        this->get_node_topics_interface(),
        this->get_node_services_interface(),
        this->get_node_parameters_interface(),
        this->get_node_timers_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_graph_interface(), 
        this->get_node_waitables_interface()
    };

    return m_pad_resource_manager_loader.createSharedInstance(plugin_name, node_interfaces_bundle);
  }

private:
  pluginlib::ClassLoader<IPadResourceManager> m_pad_resource_manager_loader;
  std::shared_ptr<IPadResourceManager> m_pad_resource_manager;
  std::shared_ptr<PadRightServer> m_pad_right_server;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PadRightActionServerNode>();
  auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node(node);
  executor->spin();
  rclcpp::shutdown();
  return 0;
}