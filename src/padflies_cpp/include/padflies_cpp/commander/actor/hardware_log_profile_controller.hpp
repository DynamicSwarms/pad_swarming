#pragma once

#include "crazyflie_interfaces/srv/add_logging.hpp"
#include "crazyflie_interfaces/srv/remove_logging.hpp"
#include "padflies_cpp/commander/actor/hardware_parameter_controller.hpp"
#include "padflies_cpp/node_interfaces_bundle.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <string>
#include <vector>

class HardwareLogProfileController
{
public:
  HardwareLogProfileController(
    const std::string & cf_prefix,
    const padflies_cpp::NodeInterfacesBundle & node_interfaces,
    std::shared_ptr<HardwareParameterController> parameter_controller);

  /// Detect the configured profiles without allocating firmware log blocks.
  void configure();

  /// Start all log blocks belonging to detected profiles.
  void activate();

  /// Best-effort removal of every block started by activate().
  void deactivate();

  void cleanup();

  const std::vector<std::string> & capabilities() const;

private:
  struct LogBlock
  {
    std::string topic;
    int frequency{};
    std::vector<std::string> variables;
  };

  struct Profile
  {
    std::string name;
    std::string capability;
    std::string detection_parameter;
    std::vector<LogBlock> blocks;
    bool detected{false};
  };

  void read_profiles();
  bool detect(const Profile & profile) const;
  bool add_block(const LogBlock & block);
  void remove_block(const std::string & topic);

  std::string m_cf_prefix;
  rclcpp::Logger m_logger;
  std::shared_ptr<HardwareParameterController> m_parameter_controller;
  std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
  std::shared_ptr<rclcpp::Client<crazyflie_interfaces::srv::AddLogging>> m_add_client;
  std::shared_ptr<rclcpp::Client<crazyflie_interfaces::srv::RemoveLogging>> m_remove_client;
  std::shared_ptr<rclcpp::node_interfaces::NodeParametersInterface> m_parameters_interface;

  std::vector<Profile> m_profiles;
  std::vector<std::string> m_capabilities;
  std::vector<std::string> m_active_topics;
};
