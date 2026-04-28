#include "behaviortree_cpp/bt_factory.h"

class WaitFor : public BT::StatefulActionNode
{
  public: 
  WaitFor(
    const std::string& name, 
    const BT::NodeConfig& config, 
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface) 
  : BT::StatefulActionNode(name, config)
  , m_clock(clock_interface->get_clock())
  {

  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<double>("duration") };
  }

  BT::NodeStatus onStart() override
  {
    BT::Expected<double> duration_exp = getInput<double>("duration");
    if (!duration_exp)    {
      std::cerr << "Error getting duration: " << duration_exp.error() << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    double duration = duration_exp.value();
    std::cout << "Waiting for " << duration << " seconds..." << std::endl;
    m_end_time = m_clock->now() + std::chrono::milliseconds(static_cast<int>(duration * 1000));
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (m_clock->now() >= m_end_time) {
      std::cout << "Done waiting!" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::cout << "WaitFor halted!" << std::endl;
  }

  private: 
  rclcpp::Time m_end_time;
  std::shared_ptr<rclcpp::Clock> m_clock;

};