#ifndef FRC846_TRAJECTORY_GENERATOR_H_
#define FRC846_TRAJECTORY_GENERATOR_H_

#include <optional>
#include <vector>

#include "frc846/math.h"

namespace frc846 {

// User-specified target waypoint along a trajectory.
struct InputWaypoint {
  frc846::Position pos;

  // Optional max velocity at point.
  std::optional<units::feet_per_second_t> v_max;
};

// Waypoint for the robot to follow.
struct Waypoint {
  frc846::Position pos;

  // Target velocity at point.
  units::feet_per_second_t v;
};

// Robot trajectory.
using Trajectory = std::vector<Waypoint>;

// Linearly interpolate points at a given distance apart between two points.
std::vector<Vector2D<units::foot_t>> InterpolatePoints(
    Vector2D<units::foot_t> start, Vector2D<units::foot_t> end,
    units::foot_t distance);

// Generate a trajectory given a list of input waypoints.
//
// The generator takes the input points, linaerly interpolates points between
// them, then sets a target velocity at each waypoint while respecting the
// robot's max acceleration and deceleration.
Trajectory GenerateTrajectory(
    std::vector<InputWaypoint> input_points,
    units::feet_per_second_t robot_max_v,
    units::feet_per_second_squared_t robot_max_acceleration,
    units::feet_per_second_squared_t robot_max_deceleration,
    units::inch_t cut = 6_in);

}  // namespace frc846

#endif  // FRC846_TRAJECTORY_GENERATOR_H_