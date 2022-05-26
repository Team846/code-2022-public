#include "robot/commands/optimal_spinup_command.h"

#include "frc846/math.h"

OptimalSpinupCommand::OptimalSpinupCommand(RobotContainer& container)
    : frc846::Named{"optimal_spinup_command"},
      shooter_(container.shooter_),
      limelight_(container.limelight_),
      drivetrain_(container.drivetrain_),
      target_(shooter_.Initialized() ? shooter_.subsystem()->preset_mid_.value()
                                     : 0_rpm) {
  AddRequirements({&shooter_});
  SetName("optimal_spinup_command");
}

void OptimalSpinupCommand::Execute() {
  if (shooter_.Initialized()) {
    if (limelight_.Initialized() && limelight_.readings().target_exists) {
      // Find optimal speed to shoot based on limelight ty
      target_ = shooter_.subsystem()->GetOptimalRPM(limelight_.readings().ty);

      // Shoot while driving compensation
      if (shooter_.subsystem()->should_compensate_rpm_.value()) {
        auto [delta_rpm, compensated_hub_pos] =
            shooter_.subsystem()->GetCompensatedRPM(
                drivetrain_.readings().velocity.Rotate(
                    limelight_.readings().tx) *
                    -1,
                limelight_.readings(),
                limelight_.subsystem()->mounting_angle_.value(),
                limelight_.subsystem()->mounting_height_.value());

        delta_rpm *= shooter_.subsystem()->shoot_fudge_.value();

        // The optimal speed to shoot is now based on limelgiht ty along with
        // our shoot while driving compensation speed
        units::revolutions_per_minute_t estimated_rpm =
            shooter_.subsystem()->GetOptimalRPM(limelight_.readings().ty) +
            delta_rpm;

        // The theoretical hub we're shooting at with drivetrain velocity
        // compensation
        units::foot_t compensated_hub_distance =
            compensated_hub_pos.Magnitude();

        // Theoretical limelight ty value based on `compensated_hub_distance`
        units::degree_t compensated_ty = shooter_.subsystem()->DistanceToTy(
            compensated_hub_distance,
            limelight_.subsystem()->mounting_angle_.value(),
            limelight_.subsystem()->mounting_height_.value());

        // How much longer is the ball theoretically spending on the air
        units::second_t time_of_flight_error =
            shooter_.subsystem()->GetTimeFlight(compensated_ty,
                                                compensated_hub_distance) -
            shooter_.subsystem()->GetTimeFlight(
                limelight_.readings().ty, limelight_.readings().hub_distance);

        if (time_of_flight_error >
            shooter_.subsystem()->max_tof_error_.value()) {
          target_ = 0_rpm;
          shooter_.Error("MAX TOF ERROR IS TOO LARGE");
        } else {
          target_ = estimated_rpm;
        }
      }
    }

    shooter_.SetTarget({target_});
  }
}

void OptimalSpinupCommand::End(bool interrupted) {
  (void)interrupted;
  if (shooter_.Initialized()) {
    shooter_.SetTargetZero();
  }
}

bool OptimalSpinupCommand::IsFinished() { return false; }