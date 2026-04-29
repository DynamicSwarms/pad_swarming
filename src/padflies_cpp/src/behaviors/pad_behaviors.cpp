#include "behaviortree_cpp/bt_factory.h"
#include "padflies_cpp/pad_control.hpp"

class AcquirePadRight : public BT::StatefulActionNode
{
  public: 
  AcquirePadRight(
    const std::string& name, 
    const BT::NodeConfig& config, 
    std::shared_ptr<PadControl> pad_control)
  : BT::StatefulActionNode(name, config)
  , m_pad_control(pad_control)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {};
  }

  BT::NodeStatus onStart() override
  {
    m_pad_control->acquire_right_async(5.0, [this](bool success) {
      this->m_response_received = true;
      if (success) {
        RCLCPP_INFO(rclcpp::get_logger("AcquirePadRight"), "Successfully acquired pad right!");
        this->m_acquired = true;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("AcquirePadRight"), "Failed to acquire pad right!");
        this->m_acquired = false;
      }
    });
    m_start_time = std::chrono::steady_clock::now();
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning() override
  {
    // Simulate acquiring pad right for 5 seconds
    if (std::chrono::steady_clock::now() - m_start_time > std::chrono::seconds(1)) {
      std::cout << "Timeout: Failed to acquire pad right!" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    if (m_acquired) {
      return BT::NodeStatus::SUCCESS;
    }

    if (m_response_received && !m_acquired) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  void onHalted() override
  {
    std::cout << "AcquirePadRight halted!" << std::endl;
  }



private:
  bool m_response_received = false;
  bool m_acquired = false;

  std::shared_ptr<PadControl> m_pad_control;
  std::chrono::steady_clock::time_point m_start_time;

};