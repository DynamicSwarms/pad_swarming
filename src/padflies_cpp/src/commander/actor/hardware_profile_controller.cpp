#include "padflies_cpp/commander/actor/hardware_profile_controller.hpp"

#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"
#include "yaml-cpp/yaml.h"

#include <chrono>

using namespace std::chrono_literals;

HardwareProfileController::HardwareProfileController(
  const std::string & prefix,
  const std::string & cf_prefix,
  const padflies_cpp::NodeInterfacesBundle & node_interfaces,
  std::shared_ptr<HardwareParameterController> parameter_controller)
: m_prefix(prefix),
  m_cf_prefix(cf_prefix),
  m_logger(node_interfaces.logging_interface->get_logger().get_child("HardwareProfiles")),
  m_parameter_controller(std::move(parameter_controller)),
  m_callback_group(node_interfaces.base_interface->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive)),
  m_parameters_interface(node_interfaces.parameters_interface),
  m_topics_interface(node_interfaces.topics_interface)
{
  m_add_client = rclcpp::create_client<crazyflie_interfaces::srv::AddLogging>(
    node_interfaces.base_interface,
    node_interfaces.graph_interface,
    node_interfaces.services_interface,
    m_cf_prefix + "/add_logging",
    rclcpp::ServicesQoS(),
    m_callback_group);
  m_remove_client = rclcpp::create_client<crazyflie_interfaces::srv::RemoveLogging>(
    node_interfaces.base_interface,
    node_interfaces.graph_interface,
    node_interfaces.services_interface,
    m_cf_prefix + "/remove_logging",
    rclcpp::ServicesQoS(),
    m_callback_group);

  read_profiles();
}

void HardwareProfileController::read_profiles()
{
  auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
  descriptor.read_only = true;
  descriptor.description =
    "YAML file defining capability detection, firmware logging, and acting topics";
  const auto config_path = m_parameters_interface->declare_parameter(
    "hardware_profiles", rclcpp::ParameterValue(""), descriptor).get<std::string>();
  if (config_path.empty()) {
    return;
  }

  try {
    const auto profiles = YAML::LoadFile(config_path)["profiles"];
    if (!profiles || !profiles.IsMap()) {
      RCLCPP_ERROR(m_logger, "Sensor logging config '%s' has no profiles map", config_path.c_str());
      return;
    }

    for (const auto & profile_entry : profiles) {
      Profile profile;
      profile.name = profile_entry.first.as<std::string>();
      const auto profile_node = profile_entry.second;
      profile.capability = profile_node["capability"].as<std::string>(profile.name);
      profile.detection_parameter = profile_node["detection_parameter"].as<std::string>("");

      const auto blocks = profile_node["blocks"];
      if (blocks && blocks.IsMap()) {
        for (const auto & block_entry : blocks) {
          const auto block_name = block_entry.first.as<std::string>();
          const auto block_node = block_entry.second;
          LogBlock block;
          block.topic = block_node["topic"].as<std::string>("");
          block.frequency = block_node["frequency"].as<int>(10);
          block.variables = block_node["variables"].as<std::vector<std::string>>(
            std::vector<std::string>{});

          if (block.topic.empty() || block.frequency <= 0 || block.variables.empty()) {
            RCLCPP_WARN(
              m_logger, "Ignoring invalid log block '%s' in profile '%s'",
              block_name.c_str(), profile.name.c_str());
            continue;
          }
          profile.blocks.push_back(std::move(block));
        }
      }

      const auto acting = profile_node["acting"];
      if (acting && acting.IsMap()) {
        for (const auto & acting_entry : acting) {
          const auto acting_name = acting_entry.first.as<std::string>();
          const auto acting_node = acting_entry.second;
          Profile::ActingTopic acting_topic;
          acting_topic.topic = acting_node["topic"].as<std::string>("");
          acting_topic.parameter = acting_node["parameter"].as<std::string>("");
          acting_topic.type = acting_node["type"].as<std::string>("");
          if (acting_topic.topic.empty() || acting_topic.parameter.empty() ||
            acting_topic.type.empty())
          {
            RCLCPP_WARN(
              m_logger, "Ignoring invalid acting topic '%s' in profile '%s'",
              acting_name.c_str(), profile.name.c_str());
            continue;
          }
          profile.acting_topics.push_back(std::move(acting_topic));
        }
      }
      m_profiles.push_back(std::move(profile));
    }
  } catch (const YAML::Exception & error) {
    RCLCPP_ERROR(
      m_logger, "Failed to load sensor logging config '%s': %s",
      config_path.c_str(), error.what());
  }
}

bool HardwareProfileController::detect(const Profile & profile) const
{
  if (profile.detection_parameter.empty()) {
    return true;
  }

  rcl_interfaces::msg::ParameterValue value;
  if (!m_parameter_controller->get_firmware_parameter(profile.detection_parameter, value)) {
    RCLCPP_WARN(
      m_logger, "Could not read detection parameter '%s' for profile '%s'",
      profile.detection_parameter.c_str(), profile.name.c_str());
    return false;
  }

  using rcl_interfaces::msg::ParameterType;
  switch (value.type) {
    case ParameterType::PARAMETER_BOOL:
      return value.bool_value;
    case ParameterType::PARAMETER_INTEGER:
      return value.integer_value != 0;
    case ParameterType::PARAMETER_DOUBLE:
      return value.double_value != 0.0;
    default:
      RCLCPP_WARN(
        m_logger, "Detection parameter '%s' has unsupported type %u",
        profile.detection_parameter.c_str(), value.type);
      return false;
  }
}

void HardwareProfileController::configure()
{
  m_capabilities.clear();
  for (auto & profile : m_profiles) {
    profile.detected = detect(profile);
    if (profile.detected) {
      m_capabilities.push_back(profile.capability);
      RCLCPP_INFO(
        m_logger, "Detected capability '%s' for %s",
        profile.capability.c_str(), m_cf_prefix.c_str());
    }
  }

  if (!m_parameters_interface->has_parameter("capabilities")) {
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor();
    descriptor.read_only = true;
    descriptor.description = "Capabilities successfully initialized from hardware_profiles";
    m_parameters_interface->declare_parameter(
      "capabilities", rclcpp::ParameterValue(m_capabilities), descriptor, true);
  }
}

bool HardwareProfileController::add_block(const LogBlock & block)
{
  if (!m_add_client->wait_for_service(500ms)) {
    RCLCPP_WARN(m_logger, "AddLogging service is unavailable for %s", m_cf_prefix.c_str());
    return false;
  }

  auto request = std::make_shared<crazyflie_interfaces::srv::AddLogging::Request>();
  request->topic_name = block.topic;
  request->frequency = block.frequency;
  request->vars = block.variables;
  auto future = m_add_client->async_send_request(request);
  if (future.wait_for(500ms) != std::future_status::ready || !future.get()->success) {
    RCLCPP_WARN(m_logger, "Failed to start firmware log topic '%s'", block.topic.c_str());
    return false;
  }
  return true;
}

void HardwareProfileController::activate()
{
  if (!m_active_topics.empty() || !m_acting_subscriptions.empty()) {
    RCLCPP_WARN(m_logger, "Hardware profiles are already active for %s", m_cf_prefix.c_str());
    return;
  }

  for (const auto & profile : m_profiles) {
    if (!profile.detected) {
      continue;
    }
    for (const auto & block : profile.blocks) {
      if (add_block(block)) {
        m_active_topics.push_back(block.topic);
      }
    }
    for (const auto & acting_topic : profile.acting_topics) {
      add_acting_topic(acting_topic);
    }
  }
}

void HardwareProfileController::add_acting_topic(
  const Profile::ActingTopic & acting_topic)
{
  const auto topic = m_prefix + "/" + acting_topic.topic;
  auto subscription_options = rclcpp::SubscriptionOptions();
  subscription_options.callback_group = m_callback_group;
  const auto set_parameter = [this, parameter_name = acting_topic.parameter](
    rcl_interfaces::msg::ParameterValue value)
    {
      rcl_interfaces::msg::Parameter parameter;
      parameter.name = parameter_name;
      parameter.value = std::move(value);
      m_parameter_controller->set_parameters({parameter});
    };

  if (acting_topic.type == "uint32") {
    m_acting_subscriptions.push_back(
      rclcpp::create_subscription<std_msgs::msg::UInt32>(
        m_topics_interface, topic, 10,
        [set_parameter](const std_msgs::msg::UInt32::SharedPtr message) {
          rcl_interfaces::msg::ParameterValue value;
          value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
          value.integer_value = message->data;
          set_parameter(std::move(value));
        }, subscription_options));
  } else if (acting_topic.type == "integer") {
    m_acting_subscriptions.push_back(
      rclcpp::create_subscription<std_msgs::msg::Int64>(
        m_topics_interface, topic, 10,
        [set_parameter](const std_msgs::msg::Int64::SharedPtr message) {
          rcl_interfaces::msg::ParameterValue value;
          value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
          value.integer_value = message->data;
          set_parameter(std::move(value));
        }, subscription_options));
  } else if (acting_topic.type == "double") {
    m_acting_subscriptions.push_back(
      rclcpp::create_subscription<std_msgs::msg::Float64>(
        m_topics_interface, topic, 10,
        [set_parameter](const std_msgs::msg::Float64::SharedPtr message) {
          rcl_interfaces::msg::ParameterValue value;
          value.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
          value.double_value = message->data;
          set_parameter(std::move(value));
        }, subscription_options));
  } else if (acting_topic.type == "bool") {
    m_acting_subscriptions.push_back(
      rclcpp::create_subscription<std_msgs::msg::Bool>(
        m_topics_interface, topic, 10,
        [set_parameter](const std_msgs::msg::Bool::SharedPtr message) {
          rcl_interfaces::msg::ParameterValue value;
          value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
          value.bool_value = message->data;
          set_parameter(std::move(value));
        }, subscription_options));
  } else if (acting_topic.type == "string") {
    m_acting_subscriptions.push_back(
      rclcpp::create_subscription<std_msgs::msg::String>(
        m_topics_interface, topic, 10,
        [set_parameter](const std_msgs::msg::String::SharedPtr message) {
          rcl_interfaces::msg::ParameterValue value;
          value.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
          value.string_value = message->data;
          set_parameter(std::move(value));
        }, subscription_options));
  } else {
    RCLCPP_WARN(
      m_logger, "Ignoring acting topic '%s': unsupported type '%s'",
      topic.c_str(), acting_topic.type.c_str());
  }
}

void HardwareProfileController::remove_block(const std::string & topic)
{
  if (!m_remove_client->wait_for_service(200ms)) {
    RCLCPP_WARN(m_logger, "RemoveLogging service is unavailable for %s", m_cf_prefix.c_str());
    return;
  }

  auto request = std::make_shared<crazyflie_interfaces::srv::RemoveLogging::Request>();
  request->topic_name = topic;
  auto future = m_remove_client->async_send_request(request);
  if (future.wait_for(500ms) != std::future_status::ready || !future.get()->success) {
    RCLCPP_WARN(m_logger, "Failed to remove firmware log topic '%s'", topic.c_str());
  }
}

void HardwareProfileController::deactivate()
{
  m_acting_subscriptions.clear();
  for (auto it = m_active_topics.rbegin(); it != m_active_topics.rend(); ++it) {
    remove_block(*it);
  }
  m_active_topics.clear();
}

void HardwareProfileController::cleanup()
{
  deactivate();
  for (auto & profile : m_profiles) {
    profile.detected = false;
  }
}

const std::vector<std::string> & HardwareProfileController::capabilities() const
{
  return m_capabilities;
}
