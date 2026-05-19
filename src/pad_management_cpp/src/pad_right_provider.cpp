#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <string>
#include <memory>

#include "pad_management_cpp/pad_right_server.hpp"
#include "pad_management_cpp/pad_resource_manager.hpp"

class PadRightActionServerNode : public rclcpp::Node
{
public:
  PadRightActionServerNode()
  : Node("megapad")
  , m_pad_resource_manager()
  , m_pad_right_server(std::make_shared<PadRightServer>(
      "megapad",
      m_pad_resource_manager,
      this->get_node_base_interface(),
      this->get_node_parameters_interface(),
      this->get_node_timers_interface(),
      this->get_node_graph_interface(),
      this->get_node_clock_interface(),
      this->get_node_waitables_interface(),
      this->get_node_logging_interface()
    ))
  {}

  

private: 
  std::shared_ptr<PadRightServer> m_pad_right_server;
  PadResourceManager m_pad_resource_manager;
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