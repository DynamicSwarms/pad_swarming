#pragma once

#include <limits>
#include <map>
#include <set>
#include <mutex>
#include <optional>

#include "pad_management_interfaces/msg/site_info.hpp"
#include "padflies_cpp/commander/padflie_tf.hpp"
#include "padflies_cpp/node_interfaces_bundle.hpp"

class SiteInfos
{
public:
  using SiteInfo = pad_management_interfaces::msg::SiteInfo;

  void update(const SiteInfo::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_site_infos[message->name] = *message;
  }

  std::map<std::string, SiteInfo> get_all() const
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_site_infos;
  }

private:
  mutable std::mutex m_mutex;
  std::map<std::string, SiteInfo> m_site_infos;
};

class SiteSelector
{
public:
  using SiteInfo = pad_management_interfaces::msg::SiteInfo;

  SiteSelector(
    padflies_cpp::NodeInterfacesBundle node_interfaces_bundle,
    std::shared_ptr<PadflieTF> padflie_tf,
    rclcpp::Logger logger)
  : m_padflie_tf(std::move(padflie_tf)),
    m_logger(logger.get_child("SiteSelector")),
    m_site_infos(std::make_shared<SiteInfos>())
  {
    m_callback_group = node_interfaces_bundle.base_interface->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = m_callback_group;
    m_site_info_subscriber = rclcpp::create_subscription<SiteInfo>(
      node_interfaces_bundle.topics_interface,
      "pad_management/site_info",
      rclcpp::QoS(10).reliable().transient_local(),
      std::bind(&SiteInfos::update, m_site_infos, std::placeholders::_1),
      options);

    m_current_site = node_interfaces_bundle.parameters_interface->declare_parameter(
      "initial_site", rclcpp::ParameterValue(""),
      rcl_interfaces::msg::ParameterDescriptor().set__read_only(true)).get<std::string>();
    m_exclude_slow_pads = node_interfaces_bundle.parameters_interface->declare_parameter(
      "exclude_slow_pads", rclcpp::ParameterValue(false)).get<bool>();
    m_exclude_fast_pads = node_interfaces_bundle.parameters_interface->declare_parameter(
      "exclude_fast_pads", rclcpp::ParameterValue(false)).get<bool>();
    m_param_callback_handle = node_interfaces_bundle.parameters_interface->add_on_set_parameters_callback(
      std::bind(&SiteSelector::m_set_parameters_callback, this, std::placeholders::_1));
  }

  void set_current_site(const std::string & site_name) {m_current_site = site_name;}

  std::optional<SiteInfo> select_takeoff_site() const
  {
    if (m_current_site.empty()) {
      RCLCPP_WARN(m_logger, "No current site set for takeoff.");
      return std::nullopt;
    }
    const auto sites = m_site_infos->get_all();
    const auto current = sites.find(m_current_site);
    if (current == sites.end()) {
      RCLCPP_WARN(m_logger, "No SiteInfo received for current site '%s'.", m_current_site.c_str());
      return std::nullopt;
    }
    return current->second;
  }

  // Temporarily pad-aware. This selection policy can become a plugin later.
  std::optional<SiteInfo> select_landing_site(
    const std::set<std::string> & excluded_sites = {}) const
  {
    const auto sites = m_site_infos->get_all();
    Eigen::Affine3d vehicle_pose;
    if (!m_padflie_tf->get_cf_pose(vehicle_pose)) {
      RCLCPP_WARN(m_logger, "Could not get Crazyflie pose.");
      return std::nullopt;
    }

    auto closest = sites.end();
    double closest_distance = std::numeric_limits<double>::max();
    for (auto site = sites.begin(); site != sites.end(); ++site) {
      const auto & info = site->second;
      if (excluded_sites.contains(info.name)) continue;
      if (!info.available || info.landing_plugin_name.empty()) continue;
      if (m_exclude_slow_pads && info.charging_speed == SiteInfo::CHARGING_SPEED_SLOW) continue;
      if (m_exclude_fast_pads && info.charging_speed == SiteInfo::CHARGING_SPEED_FAST) continue;
      for (const auto & tf_name : info.pad_tf_names) {
        Eigen::Affine3d site_pose;
        if (!m_padflie_tf->can_transform_world(tf_name) ||
          !m_padflie_tf->get_world_affine3d(tf_name, site_pose)) continue;
        const double distance = (vehicle_pose.translation() - site_pose.translation()).norm();
        if (distance < closest_distance) {
          closest_distance = distance;
          closest = site;
        }
      }
    }
    if (closest == sites.end()) {
      RCLCPP_WARN(m_logger, "No suitable landing site found.");
      return std::nullopt;
    }
    RCLCPP_INFO(m_logger, "Selected landing site: %s", closest->second.name.c_str());
    return closest->second;
  }

private:
  rcl_interfaces::msg::SetParametersResult m_set_parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & parameter : parameters) {
      if (parameter.get_name() == "exclude_slow_pads") m_exclude_slow_pads = parameter.as_bool();
      if (parameter.get_name() == "exclude_fast_pads") m_exclude_fast_pads = parameter.as_bool();
    }
    return result;
  }

  std::shared_ptr<PadflieTF> m_padflie_tf;
  rclcpp::Logger m_logger;
  std::shared_ptr<SiteInfos> m_site_infos;
  std::shared_ptr<rclcpp::CallbackGroup> m_callback_group;
  std::shared_ptr<rclcpp::Subscription<SiteInfo>> m_site_info_subscriber;
  std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> m_param_callback_handle;
  std::string m_current_site;
  bool m_exclude_slow_pads{false};
  bool m_exclude_fast_pads{false};
};
