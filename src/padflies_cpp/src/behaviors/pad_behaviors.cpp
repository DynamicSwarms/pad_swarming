#include "behaviortree_cpp/bt_factory.h"


class AcquirePadRight : public BT::StatefulActionNode
{
  public: 
  AcquirePadRight(
    const std::string& name, 
    const BT::NodeConfig& config)
  : BT::StatefulActionNode(name, config)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    m_start_time = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // Simulate acquiring pad right for 5 seconds
    if (std::chrono::steady_clock::now() - m_start_time > std::chrono::seconds(1)) {
      std::cout << "Acquired pad right!" << std::endl;
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::cout << "AcquirePadRight halted!" << std::endl;
  }



private:

  std::chrono::steady_clock::time_point m_start_time;

};