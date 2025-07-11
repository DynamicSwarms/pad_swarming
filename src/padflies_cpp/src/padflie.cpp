#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "padflies_cpp/padflie_tf.hpp"

enum class CrazyflieType {
    HARDWARE,
    WEBOTS
};

class Padflie : public rclcpp_lifecycle::LifecycleNode
{
public:
  Padflie(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("padflie", options)
  {
    RCLCPP_INFO(this->get_logger(), "Padflie node has been created.");
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.read_only = true;

    declare_parameter("id", 0xE7, param_desc);
    declare_parameter("channel", 80, param_desc);
    declare_parameter("pad_id", 0, param_desc);
    declare_parameter("type", "hardware", param_desc);
    declare_parameter("battery_voltage_empty", 3.44);
    declare_parameter("battery_voltage_charged", 4.14);

    m_cf_id = get_parameter("id").as_int();
    m_cf_channel = get_parameter("channel").as_int();
    m_pad_id = get_parameter("pad_id").as_int();

    m_prefix = "/padflie_" + std::to_string(m_cf_id);
    m_cf_prefix = "/cf" + std::to_string(m_cf_id);
  }

  /**
   * Lifecycle callbacks
   */

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_INFO(this->get_logger(), "Configuring Padflie with channel: %d, id: %d, pad_id: %d", 
                m_cf_channel, m_cf_id, m_pad_id);  

    m_padflie_tf = std::make_unique<PadflieTF>(
      shared_from_this(), 
      m_cf_prefix, 
      "world");
    m_padflie_tf->set_pad("pad_" + std::to_string(m_pad_id));

    m_is_configured = true;
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_INFO(this->get_logger(), "Activating Padflie with prefix: %s", m_prefix.c_str());
    // Activate publishers, subscribers, and services here
    // ...
    if (m_padflie_tf) {
      geometry_msgs::msg::PoseStamped pad_pose;
      if (m_padflie_tf->get_pad_pose_world(pad_pose))
      {
        RCLCPP_INFO(this->get_logger(), "Pad position: [x: %.3f, y: %.3f, z: %.3f]", 
                  pad_pose.pose.position.x, pad_pose.pose.position.y, pad_pose.pose.position.z);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to get pad position.");
      }
    }
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating Padflie with prefix: %s", m_prefix.c_str());
    // Deactivate publishers, subscribers, and services here
    // ...    
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
  uint8_t m_cf_channel;
  uint8_t m_pad_id;

  std::string m_prefix;
  std::string m_cf_prefix;

  CrazyflieType m_type;

  std::unique_ptr<PadflieTF> m_padflie_tf;
  bool m_is_configured = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  auto padflie_node = std::make_shared<Padflie>();
  executor->add_node(padflie_node->get_node_base_interface());

  executor->spin();

  rclcpp::shutdown();
  return 0;
}
