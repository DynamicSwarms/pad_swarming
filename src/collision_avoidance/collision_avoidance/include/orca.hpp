#pragma once

#include <cstddef>
#include <vector>

#include <Eigen/Dense>
#include "rclcpp/time.hpp"

struct ObjectInfo {
  Eigen::Vector2d position;
  Eigen::Vector2d preferred_velocity;
  Eigen::Vector2d velocity;
  Eigen::Vector2d calculated_velocity;
  double radius;
  double max_speed;
  rclcpp::Time last_update;
};

struct OrcaLine {
  Eigen::Vector2d point;
  Eigen::Vector2d direction;
};

std::size_t linear_program_2(
  const std::vector<OrcaLine> & lines,
  double radius,
  const Eigen::Vector2d & opt_velocity,
  bool direction_opt,
  Eigen::Vector2d & result);

void linear_program_3(
  const std::vector<OrcaLine> & lines,
  std::size_t begin_line,
  double radius,
  Eigen::Vector2d & result);

OrcaLine make_orca_line(
  const ObjectInfo & self,
  const ObjectInfo & other,
  double time_horizon,
  double time_step);
