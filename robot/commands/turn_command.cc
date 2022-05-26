#include "robot/commands/turn_command.h"

#include "frc846/math.h"
#include "robot/subsystems/drivetrain.h"
#include "robot/subsystems/limelight.h"

TurnCommand::TurnCommand(RobotContainer& container,
                         units::degree_t target_angle, bool is_limelight_aim)
    : drivetrain_(container.drivetrain_),
      limelight_(container.limelight_),
      target_angle_(target_angle),
      is_limelight_aim_(is_limelight_aim) {
  AddRequirements({&drivetrain_, &limelight_});
  SetName("drive_command");
}

void TurnCommand::Initialize() { is_done_ = false; }

void TurnCommand::Execute() {
  DrivetrainTarget drivetrain_target;
  drivetrain_target.v_x = 0_fps;
  drivetrain_target.v_y = 0_fps;
  drivetrain_target.translation_reference =
      DrivetrainTranslationReference::kField;

  // Defaults to target angle specified if limelight aim fails
  if (is_limelight_aim_) {
    if (limelight_.readings().target_exists) {
      target_angle_ =
          limelight_.readings().tx + drivetrain_.readings().pose.bearing;

      drivetrain_target.rotation = DrivetrainRotationPosition(target_angle_);
    } else {
      drivetrain_.Warn("Can't find target!");
      drivetrain_target.rotation = DrivetrainRotationPosition(target_angle_);
    }
  } else {
    drivetrain_target.rotation = DrivetrainRotationPosition(target_angle_);
  }

  // Check if turn is completed
  if (units::math::abs(target_angle_ - drivetrain_.readings().pose.bearing) <
      2_deg) {
    is_done_ = true;
  }

  drivetrain_.SetTarget(drivetrain_target);
}

void TurnCommand::End(bool) { drivetrain_.SetTargetZero(); }

bool TurnCommand::IsFinished() { return is_done_; }