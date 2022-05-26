#include "robot/commands/climber_position_command.h"

#include "frc846/wpilib/time.h"
#include "robot/subsystems/robot_container.h"

ClimberPositionCommand::ClimberPositionCommand(
    RobotContainer& container, const ClimberSetpointPref& setpoint,
    Direction allowed_direction)
    : climber_(container.climber_),
      setpoint_(setpoint),
      allowed_direction_(allowed_direction) {
  AddRequirements({&climber_});
  SetName("climber_position_command");
}

void ClimberPositionCommand::Execute() {
  if (climber_.Initialized()) {
    int arms_done = 0;

    auto control = [&](units::degree_t position,
                       units::degree_t target) -> double {
      double target_speed = 0;

      auto tolerance =
          climber_.subsystem()->position_control_tolerance_.value();

      if (position > target - tolerance && position < target + tolerance) {
        arms_done++;
      } else {
        // Simple bang-bang controller
        // TODO more complex controller
        if (position < target) {
          if (allowed_direction_ == Direction::kSwing) {
            climber_.Warn("Not allowed to swing!");
          } else {
            target_speed = +std::abs(setpoint_.speed.value());
          }
        } else if (position > target) {
          if (allowed_direction_ == Direction::kClimb) {
            climber_.Warn("Not allowed to climb!");
          } else {
            target_speed = -std::abs(setpoint_.speed.value());
          }
        }
      }

      return target_speed;
    };

    auto left_target = control(climber_.readings().left.position,
                               setpoint_.position.value());
    auto right_target = control(climber_.readings().right.position,
                                setpoint_.position.value());

    is_done_ = arms_done == 2;

    climber_.SetTarget({
        left_target,
        right_target,
    });
  }
}

void ClimberPositionCommand::End(bool interrupted) {
  (void)interrupted;
  if (climber_.Initialized()) {
    climber_.SetTargetZero();
  }
}

bool ClimberPositionCommand::IsFinished() { return is_done_; }