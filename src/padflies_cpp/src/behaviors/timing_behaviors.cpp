#include "behaviortree_cpp/bt_factory.h"

class WaitFor : public BT::StatefulActionNode
{
  public: 
  WaitFor(
    const std::string& name, 
    const BT::NodeConfig& config, 
    std::shared_ptr<rclcpp::node_interfaces::NodeClockInterface> clock_interface, 
    rclcpp::Logger logger)
  : BT::StatefulActionNode(name, config)
  , m_clock(clock_interface->get_clock())
  , m_logger(logger.get_child(name))
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
      RCLCPP_ERROR(m_logger, "Error getting duration: , %s",   duration_exp.error().c_str());
      return BT::NodeStatus::FAILURE;
    }
    double duration = duration_exp.value();
    RCLCPP_INFO(m_logger, "Waiting for %f seconds...", duration);
    m_end_time = m_clock->now() + std::chrono::milliseconds(static_cast<int>(duration * 1000));
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    if (m_clock->now() >= m_end_time) {
      RCLCPP_INFO(m_logger, "Done waiting!");
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    RCLCPP_INFO(m_logger, "WaitFor halted!");
  }

  private: 
  rclcpp::Time m_end_time;
  std::shared_ptr<rclcpp::Clock> m_clock;
  rclcpp::Logger m_logger;

};