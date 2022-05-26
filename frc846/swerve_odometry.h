#ifndef FRC846_SWERVE_ODOMETRY_H_
#define FRC846_SWERVE_ODOMETRY_H_

#include <array>

#include "frc846/math.h"
#include "units/base.h"

namespace frc846 {

// Position tracking for swerve drivetrains.
//
// TODO this is hardcoded for 4 module square drivetrains, but can be
// generalized.
class SwerveOdometry {
 private:
  static constexpr int kModuleCount = 4;

 public:
  SwerveOdometry(Position initial_pose = {{0_ft, 0_ft}, 0_deg});

  // Get the current pose.
  Position pose() const { return pose_; }

  // Update the odometry given the wheel velocities and the current bearing.
  void Update(std::array<Vector2D<units::foot_t>, kModuleCount> wheel_vecs,
              units::radian_t bearing);

  // Set the odometry point to a given value.
  void SetPoint(Vector2D<units::foot_t> point);

  // Reset pose.
  void Zero();

 private:
  Position pose_;
  std::array<units::foot_t, kModuleCount> prev_wheel_distances_ = {};
};

}  // namespace frc846

#endif  // FRC846_SWERVE_ODOMETRY_H_