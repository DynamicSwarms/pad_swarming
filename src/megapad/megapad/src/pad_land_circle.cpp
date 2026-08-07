#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "pad_management_interfaces/srv/pad_idle_target.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace megapad
{

class PadLandCircle : public rclcpp::Node
{
public:
  PadLandCircle()
  : Node("pad_circle"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    declare_bounded_parameter("radius", 1.75, 0.25, 3.0);
    declare_bounded_parameter("seperation_factor", 0.5, 0.0, 1.0);
    declare_bounded_parameter("circular_speed", 0.2, 0.0, 1.0);
    tf_frame_ = declare_parameter<std::string>("tf_frame", "pad_circle");

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    service_ = create_service<pad_management_interfaces::srv::PadIdleTarget>(
      "/megapad/pad_idle_target",
      std::bind(&PadLandCircle::calculate_target, this, std::placeholders::_1,
        std::placeholders::_2),
      rclcpp::ServicesQoS(), callback_group_);
    cleanup_timer_ = create_timer(
      std::chrono::milliseconds(200),
      std::bind(&PadLandCircle::remove_stale_agents, this), callback_group_);
  }

private:
  struct Agent
  {
    std::string name;
    Eigen::Vector3d position;
    rclcpp::Time last_update;
  };

  static constexpr std::size_t formation_point_count = 32;
  static constexpr double pi = 3.14159265358979323846;

  void declare_bounded_parameter(
    const std::string & name, double default_value, double minimum, double maximum)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    rcl_interfaces::msg::FloatingPointRange range;
    range.from_value = minimum;
    range.to_value = maximum;
    descriptor.floating_point_range.push_back(range);
    declare_parameter(name, default_value, descriptor);
  }

  void remove_stale_agents()
  {
    const auto now = get_clock()->now();
    const auto timeout = rclcpp::Duration::from_seconds(0.3);
    const std::lock_guard<std::mutex> lock(agents_mutex_);
    std::erase_if(agents_, [&now, &timeout](const Agent & agent) {
      return now < agent.last_update || now - agent.last_update > timeout;
    });
  }

  void calculate_target(
    const std::shared_ptr<pad_management_interfaces::srv::PadIdleTarget::Request> request,
    std::shared_ptr<pad_management_interfaces::srv::PadIdleTarget::Response> response)
  {
    Eigen::Vector3d local_position;
    if (!to_local_coordinates(request->position, local_position)) {
      response->target = request->position;
      return;
    }

    const auto now = get_clock()->now();
    std::vector<Agent> agents;
    {
      const std::lock_guard<std::mutex> lock(agents_mutex_);
      auto agent = std::find_if(agents_.begin(), agents_.end(), [&request](const Agent & item) {
        return item.name == request->name;
      });
      if (agent == agents_.end()) {
        agents_.push_back(Agent{request->name, local_position, now});
      } else {
        agent->position = local_position;
        agent->last_update = now;
      }
      agents = agents_;
    }

    const Eigen::Vector3d target = match_formation(request->name, agents);
    response->target.header.frame_id = tf_frame_;
    response->target.pose.position.x = target.x();
    response->target.pose.position.y = target.y();
    response->target.pose.position.z = target.z();
  }

  Eigen::Vector3d match_formation(
    const std::string & name, const std::vector<Agent> & agents) const
  {
    const double radius = get_parameter("radius").as_double();
    const double separation_factor = get_parameter("seperation_factor").as_double();
    const double circular_speed = get_parameter("circular_speed").as_double();
    const double step_size = 2.0 * pi / static_cast<double>(formation_point_count);

    std::array<Eigen::Vector3d, formation_point_count> formation;
    std::array<double, formation_point_count> steps;
    for (std::size_t i = 0; i < formation_point_count; ++i) {
      const double angle = step_size * static_cast<double>(i);
      formation[i] = {radius * std::sin(angle), radius * std::cos(angle), 0.0};
      steps[i] = angle;
    }

    std::vector<double> angles;
    std::vector<Eigen::Vector2d> unit_positions;
    angles.reserve(agents.size());
    unit_positions.reserve(agents.size());
    for (const auto & agent : agents) {
      std::array<double, formation_point_count> distances;
      for (std::size_t i = 0; i < formation_point_count; ++i) {
        distances[i] = (agent.position - formation[i]).norm();
      }
      const auto closest_it = std::min_element(distances.begin(), distances.end());
      const std::size_t closest = std::distance(distances.begin(), closest_it);
      const std::size_t next = (closest + 1) % formation_point_count;
      const std::size_t previous =
        (closest + formation_point_count - 1) % formation_point_count;
      const std::size_t second = distances[next] <= distances[previous] ? next : previous;
      const double angle = circular_interpolate(
        steps[closest], steps[second], distances[closest], distances[second]);
      angles.push_back(angle);
      unit_positions.emplace_back(std::sin(angle), std::cos(angle));
    }

    const auto requested_agent = std::find_if(
      agents.begin(), agents.end(), [&name](const Agent & agent) {return agent.name == name;});
    const std::size_t requested_index = std::distance(agents.begin(), requested_agent);

    std::vector<double> angular_differences;
    for (std::size_t i = 0; i < agents.size(); ++i) {
      if (i == requested_index) {
        continue;
      }
      double angle = clockwise_angle(unit_positions[requested_index], unit_positions[i]);
      if (angle > pi) {
        angle = -(2.0 * pi - angle);
      }
      angular_differences.push_back(angle);
    }

    double separation_urge = 0.0;
    if (!angular_differences.empty()) {
      double closest_negative = pi;
      double closest_positive = pi;
      for (const double difference : angular_differences) {
        if (difference > 0.0) {
          closest_negative = std::min(closest_negative, difference);
        } else if (difference < 0.0) {
          closest_positive = std::min(closest_positive, std::abs(difference));
        }
      }
      closest_positive = -closest_positive;
      separation_urge = (closest_positive + closest_negative) * separation_factor;
    }

    const double target_angle = angles[requested_index] + circular_speed + separation_urge;
    std::array<double, formation_point_count> differences;
    for (std::size_t i = 0; i < formation_point_count; ++i) {
      differences[i] = absolute_angle_difference(target_angle, steps[i]);
    }
    std::array<std::size_t, formation_point_count> indices;
    for (std::size_t i = 0; i < formation_point_count; ++i) {
      indices[i] = i;
    }
    std::partial_sort(indices.begin(), indices.begin() + 2, indices.end(),
      [&differences](std::size_t lhs, std::size_t rhs) {
        return differences[lhs] < differences[rhs];
      });
    const auto first = indices[0];
    const auto second = indices[1];
    const double weight_sum = differences[first] + differences[second];
    return (formation[first] * differences[second] + formation[second] * differences[first]) /
           weight_sum;
  }

  static double circular_interpolate(double a0, double a1, double w0, double w1)
  {
    const double sum = w0 + w1;
    const double x = std::sin(a0) * (w1 / sum) + std::sin(a1) * (w0 / sum);
    const double y = std::cos(a0) * (w1 / sum) + std::cos(a1) * (w0 / sum);
    const double angle = std::atan2(x, y);
    return angle > 0.0 ? angle : 2.0 * pi - std::abs(angle);
  }

  static double clockwise_angle(const Eigen::Vector2d & from, const Eigen::Vector2d & to)
  {
    const double dot = std::clamp(from.normalized().dot(to.normalized()), -1.0, 1.0);
    const double angle = std::acos(dot);
    const Eigen::Vector2d rotated_to(-to.y(), to.x());
    const double direction = std::acos(std::clamp(rotated_to.normalized().dot(from.normalized()), -1.0, 1.0));
    return direction < pi / 2.0 ? angle : 2.0 * pi - angle;
  }

  static double absolute_angle_difference(double first, double second)
  {
    double difference = std::fmod(std::abs(first - second), 2.0 * pi);
    return difference < pi ? difference : 2.0 * pi - difference;
  }

  bool to_local_coordinates(
    const geometry_msgs::msg::PoseStamped & source, Eigen::Vector3d & result) const
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(tf_frame_, source.header.frame_id, tf2::TimePointZero);
      const auto & rotation = transform.transform.rotation;
      const Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
      const Eigen::Vector3d position(
        source.pose.position.x, source.pose.position.y, source.pose.position.z);
      const Eigen::Vector3d translation(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);
      result = quaternion * position + translation;
      return true;
    } catch (const tf2::TransformException & exception) {
      RCLCPP_DEBUG(get_logger(), "%s", exception.what());
      return false;
    }
  }

  std::string tf_frame_;
  std::mutex agents_mutex_;
  std::vector<Agent> agents_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Service<pad_management_interfaces::srv::PadIdleTarget>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr cleanup_timer_;
  mutable tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace megapad

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<megapad::PadLandCircle>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
