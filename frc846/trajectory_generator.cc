#include "frc846/trajectory_generator.h"

#include <algorithm>

#include "frc846/named.h"

namespace frc846 {

std::vector<Vector2D<units::foot_t>> InterpolatePoints(
    Vector2D<units::foot_t> start, Vector2D<units::foot_t> end,
    units::foot_t cut) {
  auto distance = (end - start).Magnitude();
  int n = std::max(units::math::ceil(distance / cut).to<int>(), 1);

  std::vector<Vector2D<units::foot_t>> points(n);
  for (int i = 0; i < n; ++i) {
    double weight = (double)(i) / n;
    points[i] = {
        start.x * (1 - weight) + end.x * weight,
        start.y * (1 - weight) + end.y * weight,
    };
  }

  return points;
}

void CapAcceleration(Trajectory& trajectory,
                     units::feet_per_second_squared_t max_a) {
  for (unsigned int i = 1; i < trajectory.size(); ++i) {
    auto delta_pos =
        (trajectory[i].pos.point - trajectory[i - 1].pos.point).Magnitude();

    // v₂² = v₁² + 2aΔx
    // 2aΔx = v₂² - v₁²
    // a = (v₂² - v₁²) / (2Δx)
    auto deceleration = (units::math::pow<2>(trajectory[i].v) -
                         units::math::pow<2>(trajectory[i - 1].v)) /
                        (2 * delta_pos);
    if (deceleration > max_a) {
      // v₂² = v₁² + 2aΔx
      // v₂² = sqrt(v₁² + 2aΔx)
      trajectory[i].v = units::math::sqrt(
          units::math::pow<2>(trajectory[i - 1].v) + 2 * max_a * delta_pos);
    }
  }
}

Trajectory GenerateTrajectory(
    std::vector<InputWaypoint> input_points,
    units::feet_per_second_t robot_max_v,
    units::feet_per_second_squared_t robot_max_acceleration,
    units::feet_per_second_squared_t robot_max_deceleration,
    units::inch_t cut) {
  Named named{"trajectory_generator"};

  if (input_points.size() < 2) {
    named.Error("Not enough input points! {} points given",
                input_points.size());
    return {};
  }

  // TODO check that first v > 0

  Trajectory trajectory;

  for (unsigned int i = input_points.size() - 1; i > 0; --i) {
    auto interpolated_points = InterpolatePoints(
        input_points[i].pos.point, input_points[i - 1].pos.point, cut);
    interpolated_points.erase(interpolated_points.begin());

    trajectory.push_back({
        input_points[i].pos,
        input_points[i].v_max.value_or(robot_max_v),
    });

    for (auto point : interpolated_points) {
      trajectory.push_back({
          {point, trajectory.back().pos.bearing},
          robot_max_v,
      });
    }
  }
  trajectory.push_back({
      input_points[0].pos,
      input_points[0].v_max.value_or(robot_max_v),
  });

  CapAcceleration(trajectory, robot_max_acceleration);
  std::reverse(trajectory.begin(), trajectory.end());
  CapAcceleration(trajectory, robot_max_deceleration);

  // If any point has 0 speed, just set it to the previous speed.
  for (unsigned int i = 0; i < trajectory.size(); ++i) {
    if (trajectory[i].v == 0_fps) {
      trajectory[i].v = i == 0 ? trajectory[1].v : trajectory[i - 1].v;
    }
  }

  return trajectory;
}

}  // namespace frc846