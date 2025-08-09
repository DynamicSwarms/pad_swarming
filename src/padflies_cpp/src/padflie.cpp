#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "padflies_cpp/commander.hpp"

enum class CrazyflieType {
    HARDWARE,
    WEBOTS
};

class Padflie : public rclcpp_lifecycle::LifecycleNode
{
public:
  Padflie(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("padflie", options)
  , m_cf_id(declare_parameter("id", rclcpp::ParameterValue(0xE7), rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<int>())
  , m_prefix("/padflie" + std::to_string(m_cf_id))
  , m_cf_prefix("/cf" + std::to_string(m_cf_id))
  , m_padflie_commander(m_prefix, m_cf_prefix, this->get_node_parameters_interface())
  {
  }

  /**
   * Lifecycle callbacks
   */

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_INFO(this->get_logger(), "Configuring Padflie with id: %d", m_cf_id);  
    bool success = m_padflie_commander.on_configure(shared_from_this());
    
    if (success)
    {
      m_is_configured = true;
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    } else {
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }  
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_INFO(this->get_logger(), "Activating Padflie with prefix: %s", m_prefix.c_str());
    bool success = m_padflie_commander.on_activate(shared_from_this());
    
    if (success) 
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    else
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating Padflie with prefix: %s", m_prefix.c_str());
    m_padflie_commander.on_deactivate(shared_from_this());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down Padflie with prefix: %s", m_prefix.c_str());
    // Cleanup resources here
    // ...
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_ERROR(this->get_logger(), "Error in Padflie with prefix: %s", m_prefix.c_str());
    // Handle error state here
    // ...
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) 
  { 
    RCLCPP_INFO(this->get_logger(), "Cleaning up Padflie with prefix: %s", m_prefix.c_str());
    // Cleanup resources here
    // ...
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

private: 
  uint8_t m_cf_id;

  std::string m_prefix;
  std::string m_cf_prefix;

  PadflieCommander m_padflie_commander;

  bool m_is_configured = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto padflie_node = std::make_shared<Padflie>();
  executor.add_node(padflie_node->get_node_base_interface());
  executor.spin(); // spin_some will not work, be very careful (callback_groups do not work)
  executor.remove_node(padflie_node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
