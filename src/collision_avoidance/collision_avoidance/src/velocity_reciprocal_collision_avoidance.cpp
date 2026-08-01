/**
 * 2D Ellipse and Velocity Based collision avoidance algorithm.
*/

#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "collision_avoidance_interfaces/srv/velocity_reciprocals_collision_avoidance.hpp"
#include "orca.hpp"
#include <cmath>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <Eigen/Dense>

using std::placeholders::_1;
using std::placeholders::_2;



Eigen::Vector2d calculate_orca_velocity(
  const ObjectInfo &self,
  const std::vector<ObjectInfo> &neighbors,
  double time_horizon,
  double time_step,
  double neighbor_distance,
  bool &constrained)
{
  std::vector<OrcaLine> lines;
  lines.reserve(neighbors.size());

  const double neighbor_distance_squared =
    neighbor_distance * neighbor_distance;

  for (const ObjectInfo &other : neighbors) {
    const Eigen::Vector2d offset = other.position - self.position;

    if (offset.squaredNorm() > neighbor_distance_squared) {
      continue;
    }

    lines.push_back(
      make_orca_line(self, other, time_horizon, time_step));
  }

  Eigen::Vector2d result = self.preferred_velocity;

  const std::size_t failed_line =
    linear_program_2(
      lines,
      std::max(0.0, self.max_speed),
      self.preferred_velocity,
      false,
      result);

  if (failed_line < lines.size()) {
    linear_program_3(
      lines,
      failed_line,
      std::max(0.0, self.max_speed),
      result);
  }

  constrained =
    (result - self.preferred_velocity).squaredNorm() > 1.0e-8;

  return result;
}



class CollisionAvoidanceNode : public rclcpp::Node {
public: 
  CollisionAvoidanceNode() 
    : Node("velocity_reciprocal_collision_avoidance")
  {
    const auto read_only =
      rcl_interfaces::msg::ParameterDescriptor().set__read_only(true);
    m_time_step = this->declare_parameter("time_step", 0.1, read_only);
    m_time_horizon = this->declare_parameter("time_horizon", 1.0, read_only);
    m_neighbor_distance =
      this->declare_parameter("neighbor_distance", 5.0, read_only);
    service = this->create_service<collision_avoidance_interfaces::srv::VelocityReciprocalsCollisionAvoidance>(
      "/velocity_reciprocal_collision_avoidance",
      std::bind(&CollisionAvoidanceNode::calculate_collisions, this, _1, _2)
    );
    

    cleanup_timer = rclcpp::create_timer(
      this,
      this->get_clock(),
      std::chrono::milliseconds(200),
      std::bind(&CollisionAvoidanceNode::remove_old_objects, this)
    );
  } 
private: 
  rclcpp::Service<collision_avoidance_interfaces::srv::VelocityReciprocalsCollisionAvoidance>::SharedPtr service;
  rclcpp::TimerBase::SharedPtr cleanup_timer;
  std::unordered_map<uint8_t, ObjectInfo> active_objects;

  double m_time_step;
  double m_time_horizon;
  double m_neighbor_distance;
private: 
  void remove_old_objects() {
    RCLCPP_DEBUG(this->get_logger(), "Count: %ld", active_objects.size());
    rclcpp::Time current_time = this->now(); 
    rclcpp::Duration threshold(0, 200000000); // 0.2 seconds (200,000,000 nanoseconds)

    for (auto it = active_objects.begin(); it != active_objects.end(); ) {
          if (current_time - it->second.last_update > threshold) {
              // RCLCPP_INFO(this->get_logger(), "Removing object ID: %d", it->first);
              it = active_objects.erase(it);  // Remove object and get next iterator
          } else {
              ++it;  // Move to the next item
          }
      }
  }
  
  void calculate_collisions(
      const std::shared_ptr<collision_avoidance_interfaces::srv::VelocityReciprocalsCollisionAvoidance::Request> request,
      std::shared_ptr<collision_avoidance_interfaces::srv::VelocityReciprocalsCollisionAvoidance::Response> response) {


    uint8_t id = request->id;
    Eigen::Vector2d position(request->position.x,request->position.y);
    Eigen::Vector2d preferred_velocity(request->velocity.x,request->velocity.y);
    Eigen::Vector2d velocity;
    if (active_objects.find(id) == active_objects.end()) {
      velocity = preferred_velocity;
    } else {
      velocity = active_objects[id].velocity;
    }
    ObjectInfo self{
      position, preferred_velocity, velocity, request->radius,
      request->max_speed, this->now()};
    active_objects[id] = self;

    std::vector<ObjectInfo> neighbors;
    neighbors.reserve(
        active_objects.size() > 0
          ? active_objects.size() - 1
          : 0);

    for (const auto &[other_id, object] : active_objects) {
      if (other_id != id) {
        neighbors.push_back(object);
      }
    }

    bool constrained = false;
    const Eigen::Vector2d updated_velocity =
      calculate_orca_velocity(
        self,
        neighbors,
        m_time_horizon,
        m_time_step,
        m_neighbor_distance,
        constrained);

    active_objects[id].velocity = updated_velocity;
    response->velocity.x = updated_velocity.x();
    response->velocity.y = updated_velocity.y();
    response->velocity.z = request->velocity.z;
    response->collision = constrained;
  }


};


int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CollisionAvoidanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
