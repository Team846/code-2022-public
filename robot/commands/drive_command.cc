#include "robot/commands/drive_command.h"

#include <utility>

#include "frc846/math.h"

DriveCommand::DriveCommand(RobotContainer& container)
    : driver_(container.driver_),
      operator_(container.operator_),
      drivetrain_(container.drivetrain_),
      limelight_(container.limelight_),
      shooter_(container.shooter_),
      feeder_(container.feeder_) {
  AddRequirements({&drivetrain_});
  SetName("drive_command");
}

void DriveCommand::Execute() {
  DrivetrainTarget drivetrain_target;

  // -----BUTTON MAPPINGS-----

  // Left bumper   | robot centric translation
  // Right trigger | vision aiming
  // Right bumper  | slow drive

  bool is_robot_centric = driver_.readings().left_bumper;
  bool is_slow_drive = driver_.readings().right_bumper;
  bool is_vision_aim = driver_.readings().right_trigger;

  // -----TRANSLATION CONTROL-----

  double translate_x = frc846::HorizontalDeadband(
      driver_.readings().left_stick_x, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);
  double translate_y = frc846::HorizontalDeadband(
      driver_.readings().left_stick_y, driver_.translation_deadband_.value(), 1,
      driver_.translation_exponent_.value(), 1);

  drivetrain_target.v_x = translate_x * drivetrain_.max_speed_.value();
  drivetrain_target.v_y = translate_y * drivetrain_.max_speed_.value();

  // Slow down translation if slow mode is active
  if (is_slow_drive) {
    drivetrain_target.v_x *= drivetrain_.slow_mode_percent_.value();
    drivetrain_target.v_y *= drivetrain_.slow_mode_percent_.value();
  }

  // Robot vs field oriented translation
  drivetrain_target.translation_reference =
      is_robot_centric ? DrivetrainTranslationReference::kRobot
                       : DrivetrainTranslationReference::kField;

  // -----STEER CONTROL-----

  double steer_x = frc846::HorizontalDeadband(
      driver_.readings().right_stick_x, driver_.steer_deadband_.value(), 1,
      driver_.steer_exponent_.value(), 1);

  // If there's any manual steer input, prioritize that over vision aiming
  if (steer_x != 0) {
    auto target = steer_x * drivetrain_.max_omega();

    // Slow down steering if slow mode is active
    if (is_slow_drive) {
      target *= drivetrain_.slow_mode_percent_.value();
    }

    drivetrain_target.rotation = DrivetrainRotationVelocity(target);
  } else {
    // VISION AIM
    if (is_vision_aim) {
      units::degree_t target;

      if (limelight_.readings().target_exists) {
        units::degree_t compensation_angle = 0_deg;

        // Shoot while driving compensation
        if (shooter_.Initialized() &&
            shooter_.readings().speed >=
                shooter_.subsystem()->load_speed_threshold_.value() &&
            shooter_.subsystem()->should_compensate_rpm_.value()) {
          // The "new" position of the hub we should aim for
          // Using the current drivetrain velocity and the distance to the goal,
          // calculate how much we should offset the target drivetrain bearing
          frc846::Vector2D<units::foot_t> compensated_hub_position =
              shooter_.subsystem()
                  ->GetCompensatedRPM(
                      drivetrain_.readings().velocity.Rotate(
                          limelight_.readings().tx) *
                          -1,
                      limelight_.readings(),
                      limelight_.subsystem()->mounting_angle_.value(),
                      limelight_.subsystem()->mounting_height_.value())
                  .second;

          // TODO pretty sure this is the bug: compensated_hub_position is robot
          // relative not field relative -andy 5/12/22
          compensation_angle = compensated_hub_position.Bearing() -
                               drivetrain_.readings().pose.bearing;
        }

        // Current bearing + limelight tx + shoot while driving compensation
        target = drivetrain_.readings().pose.bearing +
                 limelight_.readings().tx +
                 compensation_angle * drivetrain_.moving_fudge.value();

        // Is the operator shooting a ball?
        auto is_shooting = operator_.readings().right_trigger ||
                           operator_.readings().right_bumper;

        // Take a snapshot when shooting (only takes snapshot on rising edge of
        // `is_shooting`)
        limelight_.SetTarget({is_shooting});

        // Also relocalize our odometry using the limelight data when shooting
        if (is_shooting) {
          drivetrain_.LocalizeOdometry(limelight_.readings().hub_distance,
                                       limelight_.readings().tx);
        }
      } else {
        // If the limelight can't see the target, aim towards where the hub is
        // according to our odometry (Hub is at the origin in the field
        // coordinate system)
        target = drivetrain_.readings().pose.point.Bearing();
      }

      drivetrain_target.rotation = DrivetrainRotationPosition(target);
    }

    // DO NOTHING
    else {
      drivetrain_target.rotation = DrivetrainRotationVelocity(0_deg_per_s);
    }
  }

  drivetrain_.SetTarget(drivetrain_target);
}

bool DriveCommand::IsFinished() { return false; }