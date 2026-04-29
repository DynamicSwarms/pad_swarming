#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"


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
  , m_padflie_commander(
      m_prefix, 
      m_cf_prefix, 
      this->get_node_base_interface(), 
      this->get_node_parameters_interface(), 
      this->get_node_timers_interface(),
      this->get_node_clock_interface(), 
      this->get_node_waitables_interface(),
      this->get_node_graph_interface(),
      this->get_node_services_interface(),
      this->get_node_logging_interface())
  {
    m_commander_health_check_timer = this->create_wall_timer(
      std::chrono::milliseconds(200),
      [this]() {
        if (!m_padflie_commander.is_healthy()) 
        {
          RCLCPP_ERROR(this->get_logger(), "Padflie Commander is not healthy, deactivating");
          m_force_deactivate = true;
          this->deactivate();
        }
      });

    m_get_state_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    m_cf_get_state_client = this->create_client<lifecycle_msgs::srv::GetState>(m_cf_prefix + "/get_state", rmw_qos_profile_services_default, m_get_state_callback_group);
    m_cf_state_check_timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&Padflie::m_cf_state_check_callback, this));
    m_cf_transition_event_sub = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>(
      m_cf_prefix + "/transition_event", 10,
      std::bind(&Padflie::m_cf_transition_event_callback, this, _1));
      // Lifecycle sub and healthy check has explicitely the default callback group, so lifecycle transitions are mutually exclusive
  }

  void m_cf_transition_event_callback(const lifecycle_msgs::msg::TransitionEvent::SharedPtr msg)
  {
    if (msg->goal_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE && !m_is_configured)
    {
      RCLCPP_DEBUG(this->get_logger(), "Crazyflie started, activating commander");
      this->configure();
    } else if (msg->goal_state.id == lifecycle_msgs::msg::State::TRANSITION_STATE_SHUTTINGDOWN)
    {
      RCLCPP_DEBUG(this->get_logger(), "Crazyflie is shutting down, deactivating commander");
      if (this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
      {
        m_force_deactivate = true; // Force deactivation
        this->deactivate();
      }      
    }
  }

  // There are possibilities where thetransitionevent callback is not called because the graph update is delayed and therefore no connection between cf and padflie exists during the transition.
  // If transition event would be latching this would not be a problem, but it is not.
  void m_cf_state_check_callback()
  {
    if (m_is_configured) return; // No need to check if already configured
    
    if (!m_cf_get_state_client->wait_for_service(std::chrono::milliseconds(10)))
    {
      return;
    }

    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();
    auto future = m_cf_get_state_client->async_send_request(request);

    // Wait for the result.
    auto status = future.wait_for(std::chrono::milliseconds(50));
    if (status == std::future_status::ready)
    {
      auto response = future.get();
      if (response->current_state.id == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE && !m_is_configured)
      {
        RCLCPP_INFO(this->get_logger(), "Crazyflie started, activating commander. We missed the transition event?");
        this->configure();
      }
    } 
  }


  /**
   * Lifecycle callbacks
   */

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn 
  on_configure(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_INFO(this->get_logger(), "Configuring Padflie with id: %d", m_cf_id); 
    
    bool success;
    if (m_is_configured)
    {
      success = true; // Maybe inform the commander??
    } else {
      try {
        m_padflie_commander.on_configure(shared_from_this());
        success = true;
      } catch (const CommanderException & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to configure PadflieCommander: %s", e.what());
        success = false;
      }
    }
    
    if (success)
    {
      m_force_deactivate = false;
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
    bool success = false;
    try {
      m_padflie_commander.on_activate(shared_from_this());
      success = true;
    } catch (const CommanderException & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to activate PadflieCommander: %s", e.what());
      success = false;
    }
    
    if (success) 
    {
      m_force_deactivate = false;
      return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    } else return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_INFO(this->get_logger(), "Deactivating Padflie with prefix: %s", m_prefix.c_str());
    try {
      m_padflie_commander.on_deactivate(shared_from_this(), m_force_deactivate);
    } catch (const CommanderException & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to deactivate PadflieCommander: %s", e.what());
      //return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }
   
   
    m_force_deactivate = false; // Reset the force deactivate flag
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_INFO(this->get_logger(), "Padflie doesnt suport shutdonw");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &) 
  {
    RCLCPP_ERROR(this->get_logger(), "Padflie doesnt suport error handling");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) 
  { 
    RCLCPP_INFO(this->get_logger(), "Cleanup not supported for padflies");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

private: 
  uint8_t m_cf_id;

  std::string m_prefix;
  std::string m_cf_prefix;

  PadflieCommander m_padflie_commander;

  std::shared_ptr<rclcpp::CallbackGroup> m_get_state_callback_group;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> m_cf_get_state_client;
  std::shared_ptr<rclcpp::TimerBase> m_cf_state_check_timer;
  std::shared_ptr<rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>> m_cf_transition_event_sub;
  std::shared_ptr<rclcpp::TimerBase> m_commander_health_check_timer;

  bool m_is_configured = false;

  bool m_force_deactivate = false; // Used to force deactivation when the Crazyflie is shutting down
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
