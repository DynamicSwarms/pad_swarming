#include "orca.hpp"

#include <algorithm>
#include <cmath>

double det(const Eigen::Vector2d & a, const Eigen::Vector2d & b) {
    return a.x() * b.y() - a.y() * b.x();
}

double kEpsilon = 1e-9;
Eigen::Vector2d normalize_or_zero(const Eigen::Vector2d & v)
{
    double norm = v.norm();
    if (norm < kEpsilon) {
        return Eigen::Vector2d(0, 0);
    }
    return v / norm;
}

/*
 * Finds the best point on one line while satisfying all earlier lines and the
 * maximum-speed circle.
 */
bool linear_program_1(
  const std::vector<OrcaLine> &lines,
  std::size_t line_no,
  double radius,
  const Eigen::Vector2d &opt_velocity,
  bool direction_opt,
  Eigen::Vector2d &result)
{
  const OrcaLine &line = lines[line_no];

  const double dot_product = line.point.dot(line.direction);
  const double discriminant =
    dot_product * dot_product + radius * radius - line.point.squaredNorm();

  if (discriminant < 0.0) {
    return false;
  }

  const double sqrt_discriminant = std::sqrt(discriminant);
  double t_left = -dot_product - sqrt_discriminant;
  double t_right = -dot_product + sqrt_discriminant;

  for (std::size_t i = 0; i < line_no; ++i) {
    const double denominator = det(line.direction, lines[i].direction);
    const double numerator =
      det(lines[i].direction, line.point - lines[i].point);

    if (std::abs(denominator) <= kEpsilon) {
      if (numerator < 0.0) {
        return false;
      }
      continue;
    }

    const double t = numerator / denominator;

    if (denominator >= 0.0) {
      t_right = std::min(t_right, t);
    } else {
      t_left = std::max(t_left, t);
    }

    if (t_left > t_right) {
      return false;
    }
  }

  double t = 0.0;

  if (direction_opt) {
    t = opt_velocity.dot(line.direction) > 0.0 ? t_right : t_left;
  } else {
    t = line.direction.dot(opt_velocity - line.point);
    t = std::clamp(t, t_left, t_right);
  }

  result = line.point + t * line.direction;
  return true;
}

/*
 * Incremental 2D linear program.
 *
 * Returns lines.size() on success, otherwise the index of the first failed line.
 */
std::size_t linear_program_2(
  const std::vector<OrcaLine> &lines,
  double radius,
  const Eigen::Vector2d &opt_velocity,
  bool direction_opt,
  Eigen::Vector2d &result)
{
  if (direction_opt) {
    result = opt_velocity * radius;
  } else if (opt_velocity.squaredNorm() > radius * radius) {
    result = normalize_or_zero(opt_velocity) * radius;
  } else {
    result = opt_velocity;
  }

  for (std::size_t i = 0; i < lines.size(); ++i) {
    // Positive means result lies outside the permitted half-plane.
    if (det(lines[i].direction, lines[i].point - result) > 0.0) {
      const Eigen::Vector2d previous_result = result;

      if (!linear_program_1(
            lines, i, radius, opt_velocity, direction_opt, result))
      {
        result = previous_result;
        return i;
      }
    }
  }

  return lines.size();
}

/*
 * Resolves infeasible combinations by minimizing penetration into the agent
 * constraints. This is the standard ORCA fallback used by RVO2.
 */
void linear_program_3(
  const std::vector<OrcaLine> &lines,
  std::size_t begin_line,
  double radius,
  Eigen::Vector2d &result)
{
  double distance = 0.0;

  for (std::size_t i = begin_line; i < lines.size(); ++i) {
    if (det(lines[i].direction, lines[i].point - result) <= distance) {
      continue;
    }

    std::vector<OrcaLine> projected_lines;
    projected_lines.reserve(i);

    for (std::size_t j = 0; j < i; ++j) {
      OrcaLine projected;

      const double determinant =
        det(lines[i].direction, lines[j].direction);

      if (std::abs(determinant) <= kEpsilon) {
        if (lines[i].direction.dot(lines[j].direction) > 0.0) {
          continue;
        }

        projected.point = 0.5 * (lines[i].point + lines[j].point);
      } else {
        projected.point =
          lines[i].point +
          (
            det(lines[j].direction, lines[i].point - lines[j].point) /
            determinant
          ) * lines[i].direction;
      }

      projected.direction =
        normalize_or_zero(lines[j].direction - lines[i].direction);

      projected_lines.push_back(projected);
    }

    const Eigen::Vector2d previous_result = result;

    const Eigen::Vector2d optimization_direction(
      -lines[i].direction.y(),
      lines[i].direction.x());

    if (linear_program_2(
          projected_lines,
          radius,
          optimization_direction,
          true,
          result) < projected_lines.size())
    {
      // Floating-point failure: retain the previous best result.
      result = previous_result;
    }

    distance = det(lines[i].direction, lines[i].point - result);
  }
}

OrcaLine make_orca_line(
  const ObjectInfo &self,
  const ObjectInfo &other,
  double time_horizon,
  double time_step)
{
  const Eigen::Vector2d relative_position =
    other.position - self.position;
  const Eigen::Vector2d relative_velocity =
    self.velocity - other.velocity;

  const double distance_squared = relative_position.squaredNorm();
  const double combined_radius = self.radius + other.radius;
  const double combined_radius_squared = combined_radius * combined_radius;

  OrcaLine line;
  Eigen::Vector2d correction = Eigen::Vector2d::Zero();

  if (distance_squared > combined_radius_squared) {
    // Agents are currently separated.
    const double inverse_time_horizon = 1.0 / time_horizon;

    const Eigen::Vector2d w =
      relative_velocity - inverse_time_horizon * relative_position;

    const double w_length_squared = w.squaredNorm();
    const double dot_product = w.dot(relative_position);

    if (
      dot_product < 0.0 &&
      dot_product * dot_product >
        combined_radius_squared * w_length_squared)
    {
      // Project onto the cut-off circle.
      const double w_length = std::sqrt(w_length_squared);
      Eigen::Vector2d unit_w;
      if (w_length > kEpsilon) {
        unit_w = w / w_length;
      } else {
        unit_w = -normalize_or_zero(relative_position);
      }

      line.direction =
        Eigen::Vector2d(unit_w.y(), -unit_w.x());

      correction =
        (combined_radius * inverse_time_horizon - w_length) * unit_w;
    } else {
      // Project onto one of the cone legs.
      const double leg =
        std::sqrt(std::max(
          0.0, distance_squared - combined_radius_squared));

      if (det(relative_position, w) > 0.0) {
        line.direction = Eigen::Vector2d(
          relative_position.x() * leg -
            relative_position.y() * combined_radius,
          relative_position.x() * combined_radius +
            relative_position.y() * leg) / distance_squared;
      } else {
        line.direction = -Eigen::Vector2d(
          relative_position.x() * leg +
            relative_position.y() * combined_radius,
          -relative_position.x() * combined_radius +
            relative_position.y() * leg) / distance_squared;
      }

      correction =
        relative_velocity.dot(line.direction) * line.direction -
        relative_velocity;
    }
  } else {
    // Agents overlap or touch. Resolve within one simulation step.
    const double inverse_time_step = 1.0 / time_step;

    const Eigen::Vector2d w =
      relative_velocity - inverse_time_step * relative_position;

    const double w_length = w.norm();
    Eigen::Vector2d unit_w;

    if (w_length > kEpsilon) {
      unit_w = w / w_length;
    } else if (relative_position.squaredNorm() > kEpsilon) {
      unit_w = -relative_position.normalized();
    } else {
      // Exactly coincident agents: select a deterministic separating direction.
      unit_w = Eigen::Vector2d(1.0, 0.0);
    }

    line.direction =
      Eigen::Vector2d(unit_w.y(), -unit_w.x());

    correction =
      (combined_radius * inverse_time_step - w_length) * unit_w;
  }

  // Reciprocal responsibility: self applies half of the required correction.
  line.point = self.velocity + 0.5 * correction;
  line.direction = normalize_or_zero(line.direction);

  return line;
}
