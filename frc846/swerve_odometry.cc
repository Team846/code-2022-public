#include "frc846/swerve_odometry.h"

#include <cmath>
#include <cstdio>

#include "frc846/math.h"

namespace frc846 {

SwerveOdometry::SwerveOdometry(Position initial_pose) : pose_(initial_pose) {}

void SwerveOdometry::Update(
    std::array<Vector2D<units::foot_t>, kModuleCount> wheel_vecs,
    units::radian_t bearing) {
  // change in distance from the last odometry update
  for (int i = 0; i < kModuleCount; i++) {
    units::foot_t wheel_dist = wheel_vecs[i].Magnitude();
    units::foot_t delta_distance = wheel_dist - prev_wheel_distances_[i];
    units::foot_t dx =
        delta_distance * units::math::cos(wheel_vecs[i].Bearing() + bearing);
    units::foot_t dy =
        delta_distance * units::math::sin(wheel_vecs[i].Bearing() + bearing);

    prev_wheel_distances_[i] = wheel_dist;

    wheel_vecs[i] = {dx, dy};
  }

  // get the distance components of each of the module, accounting for the robot
  // bearing
  std::array<Vector2D<units::foot_t>, kModuleCount> xy_comps;
  for (int i = 0; i < kModuleCount; i++) {
    xy_comps[i] = {
        wheel_vecs[i].Magnitude() * units::math::sin(wheel_vecs[i].Bearing()),
        wheel_vecs[i].Magnitude() * units::math::cos(wheel_vecs[i].Bearing()),
    };
  }

  // the distance travelled by each "side" of the robot
  units::foot_t top = (xy_comps[0].x + xy_comps[1].x) / 2;
  units::foot_t bottom = (xy_comps[3].x + xy_comps[2].x) / 2;
  units::foot_t left = (xy_comps[0].y + xy_comps[2].y) / 2;
  units::foot_t right = (xy_comps[1].y + xy_comps[3].y) / 2;

  units::radian_t theta = bearing - pose_.bearing;

  auto sin_theta = units::math::sin(theta);
  auto cos_theta = units::math::cos(theta);

  auto top_bottom = Vector2D<units::foot_t>{
      (left + right) * sin_theta,
      (left + right) * cos_theta,
  };
  auto left_right = Vector2D<units::foot_t>{
      (top + bottom) * cos_theta,
      (top + bottom) * -sin_theta,
  };

  pose_.point.x += (top_bottom.y + left_right.y) / 2;
  pose_.point.y += (top_bottom.x + left_right.x) / 2;
  pose_.bearing = bearing;
}

void SwerveOdometry::SetPoint(Vector2D<units::foot_t> point) {
  pose_.point = point;
}

void SwerveOdometry::Zero() {
  SetPoint({0_ft, 0_ft});
  pose_.bearing = 0_deg;
}

}  // namespace frc846