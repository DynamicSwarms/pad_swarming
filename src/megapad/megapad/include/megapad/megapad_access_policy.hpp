#pragma once

#include <vector>

#include <Eigen/Core>

namespace megapad
{

struct AccessGeometry2D
{
    /// Current Crazyflie position in the common world coordinate system, in meters.
    Eigen::Vector2d crazyflie_position;

    /// Position assigned to this Crazyflie in the same coordinate system, in meters.
    Eigen::Vector2d associated_position;
};

/// Evaluates whether Crazyflie-to-pad paths can be active at the same time.
///
/// The policy is independent of ROS and access-request bookkeeping. All inputs
/// must already be transformed into one common 2D coordinate system.
class MegaPadAccessPolicy
{
public:
    /// Returns the associated positions blocked by one access path.
    ///
    /// The path is the finite line segment from `access.crazyflie_position` to
    /// `access.associated_position`. An associated position from `all_accesses`
    /// is blocked when its shortest distance to that segment is less than 0.3 m.
    /// Duplicate associated positions are considered only once.
    std::vector<Eigen::Vector2d> marked_associated_positions(
        const AccessGeometry2D & access,
        const std::vector<AccessGeometry2D> & all_accesses) const;

    /// Returns positions that the requester and at least one holder both block.
    ///
    /// Every path is evaluated against the same `all_accesses` position set, so
    /// a holder's blocked region remains stable while different requests are
    /// checked. Each conflicting position is returned at most once.
    std::vector<Eigen::Vector2d> blocked_associated_positions(
        const AccessGeometry2D & requesting,
        const std::vector<AccessGeometry2D> & current_holders,
        const std::vector<AccessGeometry2D> & all_accesses) const;

    /// Returns true when the requester would not double-block any position.
    ///
    /// Access is allowed when `blocked_associated_positions()` is empty and the
    /// requester's path does not intersect any current holder's path. Touching
    /// endpoints and collinear overlap count as intersections.
    bool can_grant(
        const AccessGeometry2D & requesting,
        const std::vector<AccessGeometry2D> & current_holders,
        const std::vector<AccessGeometry2D> & all_accesses) const;

private:
    /// Tests two closed 2D line segments for crossing, touching, or overlap.
    static bool paths_intersect(
        const AccessGeometry2D & first,
        const AccessGeometry2D & second);
};

}  // namespace megapad
