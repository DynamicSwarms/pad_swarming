#include "velocity_reciprocal_visualizer.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "visualization_msgs/msg/marker.hpp"

namespace
{
constexpr std::size_t kMaximumTrailPoints = 200;
constexpr double kPreferredArrowShaftDiameter = 0.025;
constexpr double kPreferredArrowHeadDiameter = 0.075;
constexpr double kPreferredArrowHeadLength = 0.08;
constexpr double kMinimumReferenceSpeed = 0.10;
constexpr float kMinimumBodyOpacity = 0.10F;
constexpr double kMarkerLifetimeSeconds = 0.5;
constexpr const char * kMarkerNamespaces[] = {
  "orca_agents",
  "orca_preferred_velocity",
  "orca_safe_velocity",
  "orca_trails",
  "orca_ids"};
}

VelocityReciprocalVisualizer::VelocityReciprocalVisualizer(rclcpp::Node & node)
: publisher_(node.create_publisher<visualization_msgs::msg::MarkerArray>(
    "visualization_marker_array", 10)),
  clock_(node.get_clock())
{
}

void VelocityReciprocalVisualizer::publish(
  const std::unordered_map<uint8_t, ObjectInfo> & objects,
  std::optional<uint8_t> updated_id)
{
  for (const auto & [id, object] : objects) {
    if (updated_id && id != *updated_id) continue;
    auto & trail = trails_[id];
    trail.push_back(point(object.position.x(), object.position.y()));
    if (trail.size() > kMaximumTrailPoints) {
      trail.pop_front();
    }
  }

  for (auto iterator = trails_.begin(); iterator != trails_.end();) {
    if (objects.find(iterator->first) == objects.end()) {
      iterator = trails_.erase(iterator);
    } else {
      ++iterator;
    }
  }

  visualization_msgs::msg::MarkerArray array;
  array.markers.reserve(objects.size() * 5 + published_ids_.size() * 5);
  const auto stamp = clock_->now();

  for (const auto id : published_ids_) {
    if (objects.find(id) != objects.end()) continue;
    for (const auto * marker_namespace : kMarkerNamespaces) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = stamp;
      marker.ns = marker_namespace;
      marker.id = static_cast<int32_t>(id);
      marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(std::move(marker));
    }
  }

  published_ids_.clear();
  for (const auto & [id, object] : objects) {
    (void)object;
    published_ids_.insert(id);
  }

  for (const auto & [id, object] : objects) {
    const auto color = color_for_id(id);
    const auto marker_id = static_cast<int32_t>(id);

    visualization_msgs::msg::Marker body;
    body.header.frame_id = "world";
    body.header.stamp = stamp;
    body.ns = "orca_agents";
    body.id = marker_id;
    body.type = visualization_msgs::msg::Marker::SPHERE;
    body.action = visualization_msgs::msg::Marker::ADD;
    body.lifetime = rclcpp::Duration::from_seconds(kMarkerLifetimeSeconds);
    body.pose.position = point(object.position.x(), object.position.y());
    body.pose.orientation.w = 1.0;
    body.scale.x = 2.0 * object.radius;
    body.scale.y = 2.0 * object.radius;
    body.scale.z = 2.0 * object.radius;
    body.color.r = color.red;
    body.color.g = color.green;
    body.color.b = color.blue;
    const double velocity_change = (object.velocity - object.preferred_velocity).norm();
    const double reference_speed = std::max(
      object.preferred_velocity.norm(), kMinimumReferenceSpeed);
    const double relative_change = std::clamp(
      velocity_change / reference_speed, 0.0, 1.0);
    // Square-root scaling makes small but meaningful corrections visible.
    const float intervention = static_cast<float>(std::sqrt(relative_change));
    body.color.a = kMinimumBodyOpacity + (1.0F - kMinimumBodyOpacity) * intervention;
    array.markers.push_back(body);

    visualization_msgs::msg::Marker preferred;
    preferred.header = body.header;
    preferred.ns = "orca_preferred_velocity";
    preferred.id = marker_id;
    preferred.type = visualization_msgs::msg::Marker::ARROW;
    preferred.action = visualization_msgs::msg::Marker::ADD;
    preferred.lifetime = body.lifetime;
    preferred.points = {
      point(object.position.x(), object.position.y(), 0.02),
      point(
        object.position.x() + object.preferred_velocity.x(),
        object.position.y() + object.preferred_velocity.y(), 0.02)};
    preferred.scale.x = kPreferredArrowShaftDiameter;
    preferred.scale.y = kPreferredArrowHeadDiameter;
    preferred.scale.z = kPreferredArrowHeadLength;
    preferred.color.r = 0.10F;
    preferred.color.g = 0.55F;
    preferred.color.b = 1.0F;
    preferred.color.a = 0.55F;
    array.markers.push_back(preferred);

    visualization_msgs::msg::Marker safe = preferred;
    safe.ns = "orca_safe_velocity";
    safe.points[0] = point(
      object.position.x(), object.position.y(), 0.05);
    safe.points[1] = point(
      object.position.x() + object.velocity.x(),
      object.position.y() + object.velocity.y(), 0.05);
    safe.scale.x = kPreferredArrowShaftDiameter;
    safe.scale.y = kPreferredArrowHeadDiameter;
    safe.scale.z = kPreferredArrowHeadLength;
    safe.color.r = 1.0F;
    safe.color.g = 0.0F;
    safe.color.b = 0.0F;
    safe.color.a = 1.0F;
    array.markers.push_back(safe);

    visualization_msgs::msg::Marker trail;
    trail.header = body.header;
    trail.ns = "orca_trails";
    trail.id = marker_id;
    trail.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trail.action = visualization_msgs::msg::Marker::ADD;
    trail.lifetime = body.lifetime;
    trail.pose.orientation.w = 1.0;
    trail.scale.x = 0.025;
    trail.color.r = color.red;
    trail.color.g = color.green;
    trail.color.b = color.blue;
    trail.color.a = 0.65F;
    const auto trail_iterator = trails_.find(id);
    if (trail_iterator != trails_.end()) {
      trail.points.assign(trail_iterator->second.begin(), trail_iterator->second.end());
    }
    array.markers.push_back(trail);

    visualization_msgs::msg::Marker label;
    label.header = body.header;
    label.ns = "orca_ids";
    label.id = marker_id;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.lifetime = body.lifetime;
    label.pose.position = point(
      object.position.x(), object.position.y(), object.radius + 0.15);
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.18;
    label.color.r = color.red;
    label.color.g = color.green;
    label.color.b = color.blue;
    label.color.a = 1.0F;
    label.text = std::to_string(id);
    array.markers.push_back(label);
  }

  publisher_->publish(array);
}

void VelocityReciprocalVisualizer::clear()
{
  if (!publisher_) return;
  visualization_msgs::msg::MarkerArray array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = clock_->now();
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  array.markers.push_back(std::move(marker));
  publisher_->publish(array);
  published_ids_.clear();
  trails_.clear();
}

VelocityReciprocalVisualizer::Color
VelocityReciprocalVisualizer::color_for_id(uint8_t id)
{
  // Golden-ratio hue stepping spreads consecutive IDs around the spectrum.
  const double hue = std::fmod(static_cast<double>(id) * 0.61803398875, 1.0);
  const double scaled = hue * 6.0;
  const int sector = static_cast<int>(std::floor(scaled));
  const double fraction = scaled - sector;
  constexpr double saturation = 0.78;
  constexpr double value = 1.0;
  const double p = value * (1.0 - saturation);
  const double q = value * (1.0 - saturation * fraction);
  const double t = value * (1.0 - saturation * (1.0 - fraction));

  switch (sector % 6) {
    case 0: return {1.0F, static_cast<float>(t), static_cast<float>(p)};
    case 1: return {static_cast<float>(q), 1.0F, static_cast<float>(p)};
    case 2: return {static_cast<float>(p), 1.0F, static_cast<float>(t)};
    case 3: return {static_cast<float>(p), static_cast<float>(q), 1.0F};
    case 4: return {static_cast<float>(t), static_cast<float>(p), 1.0F};
    default: return {1.0F, static_cast<float>(p), static_cast<float>(q)};
  }
}

geometry_msgs::msg::Point VelocityReciprocalVisualizer::point(
  double x, double y, double z)
{
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}
