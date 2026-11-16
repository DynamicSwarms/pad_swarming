#include "megapad/megapad_access_policy.hpp"

#include <algorithm>
#include <cmath>

namespace megapad
{

std::vector<Eigen::Vector2d> MegaPadAccessPolicy::marked_associated_positions(
    const AccessGeometry2D & access,
    const std::vector<AccessGeometry2D> & all_accesses) const
{
    constexpr double marking_radius_m = 0.25;
    constexpr double minimum_segment_length_squared = 1e-12;
    const Eigen::Vector2d segment =
        access.associated_position - access.crazyflie_position;
    const double segment_length_squared = segment.squaredNorm();

    std::vector<Eigen::Vector2d> associated_positions;
    associated_positions.reserve(all_accesses.size());
    for (const auto & candidate : all_accesses) {
        const bool already_present = std::any_of(
            associated_positions.begin(), associated_positions.end(),
            [&candidate](const Eigen::Vector2d & position) {
                return position.isApprox(candidate.associated_position);
            });
        if (!already_present) {
            associated_positions.push_back(candidate.associated_position);
        }
    }

    std::vector<Eigen::Vector2d> marked;
    for (const auto & associated_position : associated_positions) {
        double interpolation = 0.0;
        if (segment_length_squared > minimum_segment_length_squared) {
            interpolation = std::clamp(
                (associated_position - access.crazyflie_position).dot(segment) /
                    segment_length_squared,
                0.0, 1.0);
        }
        const Eigen::Vector2d closest_point =
            access.crazyflie_position + interpolation * segment;
        if ((associated_position - closest_point).norm() < marking_radius_m) {
            marked.push_back(associated_position);
        }
    }
    return marked;
}

std::vector<Eigen::Vector2d> MegaPadAccessPolicy::blocked_associated_positions(
    const AccessGeometry2D & requesting,
    const std::vector<AccessGeometry2D> & current_holders,
    const std::vector<AccessGeometry2D> & all_accesses) const
{
    const auto requesting_marks =
        marked_associated_positions(requesting, all_accesses);
    std::vector<Eigen::Vector2d> blocked;
    for (const auto & holder : current_holders) {
        const auto holder_marks = marked_associated_positions(holder, all_accesses);
        for (const auto & requested_position : requesting_marks) {
            const bool is_double_marked = std::any_of(
                holder_marks.begin(), holder_marks.end(),
                [&requested_position](const Eigen::Vector2d & holder_position) {
                    return requested_position.isApprox(holder_position);
                });
            const bool already_recorded = std::any_of(
                blocked.begin(), blocked.end(),
                [&requested_position](const Eigen::Vector2d & blocked_position) {
                    return requested_position.isApprox(blocked_position);
                });
            if (is_double_marked && !already_recorded) {
                blocked.push_back(requested_position);
            }
        }
    }
    return blocked;
}

bool MegaPadAccessPolicy::can_grant(
    const AccessGeometry2D & requesting,
    const std::vector<AccessGeometry2D> & current_holders,
    const std::vector<AccessGeometry2D> & all_accesses) const
{
    if (!blocked_associated_positions(
            requesting, current_holders, all_accesses).empty())
    {
        return false;
    }

    return std::none_of(
        current_holders.begin(), current_holders.end(),
        [&requesting](const AccessGeometry2D & holder) {
            return paths_intersect(requesting, holder);
        });
}

bool MegaPadAccessPolicy::paths_intersect(
    const AccessGeometry2D & first,
    const AccessGeometry2D & second)
{
    constexpr double epsilon = 1e-9;
    const auto cross = [](const Eigen::Vector2d & left, const Eigen::Vector2d & right) {
        return left.x() * right.y() - left.y() * right.x();
    };
    const auto orientation = [&cross](
        const Eigen::Vector2d & start,
        const Eigen::Vector2d & end,
        const Eigen::Vector2d & point)
    {
        return cross(end - start, point - start);
    };
    const auto lies_on_segment = [epsilon](
        const Eigen::Vector2d & start,
        const Eigen::Vector2d & end,
        const Eigen::Vector2d & point)
    {
        return point.x() >= std::min(start.x(), end.x()) - epsilon &&
               point.x() <= std::max(start.x(), end.x()) + epsilon &&
               point.y() >= std::min(start.y(), end.y()) - epsilon &&
               point.y() <= std::max(start.y(), end.y()) + epsilon;
    };

    const auto & first_start = first.crazyflie_position;
    const auto & first_end = first.associated_position;
    const auto & second_start = second.crazyflie_position;
    const auto & second_end = second.associated_position;
    const double first_start_side = orientation(first_start, first_end, second_start);
    const double first_end_side = orientation(first_start, first_end, second_end);
    const double second_start_side = orientation(second_start, second_end, first_start);
    const double second_end_side = orientation(second_start, second_end, first_end);

    if (((first_start_side > epsilon && first_end_side < -epsilon) ||
         (first_start_side < -epsilon && first_end_side > epsilon)) &&
        ((second_start_side > epsilon && second_end_side < -epsilon) ||
         (second_start_side < -epsilon && second_end_side > epsilon)))
    {
        return true;
    }

    return (std::abs(first_start_side) <= epsilon &&
            lies_on_segment(first_start, first_end, second_start)) ||
           (std::abs(first_end_side) <= epsilon &&
            lies_on_segment(first_start, first_end, second_end)) ||
           (std::abs(second_start_side) <= epsilon &&
            lies_on_segment(second_start, second_end, first_start)) ||
           (std::abs(second_end_side) <= epsilon &&
            lies_on_segment(second_start, second_end, first_end));
}

}  // namespace megapad
