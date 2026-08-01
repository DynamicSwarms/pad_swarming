#pragma once

#include <cstdint>
#include <deque>
#include <unordered_map>

#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "orca.hpp"

class VelocityReciprocalVisualizer
{
public:
  explicit VelocityReciprocalVisualizer(rclcpp::Node & node);

  void publish(
    const std::unordered_map<uint8_t, ObjectInfo> & objects,
    uint8_t updated_id);

private:
  struct Color
  {
    float red;
    float green;
    float blue;
  };

  static Color color_for_id(uint8_t id);
  static geometry_msgs::msg::Point point(double x, double y, double z = 0.0);

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::Clock::SharedPtr clock_;
  std::unordered_map<uint8_t, std::deque<geometry_msgs::msg::Point>> trails_;
};
