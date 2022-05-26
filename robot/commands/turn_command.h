#ifndef ROBOT_COMMANDS_TURN_COMMAND_H_
#define ROBOT_COMMANDS_TURN_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "robot/subsystems/driver.h"
#include "robot/subsystems/drivetrain.h"
#include "robot/subsystems/limelight.h"
#include "robot/subsystems/robot_container.h"

// Turns the robot to a specified angle, optionally using limelight aim if a
// target exists as well.
class TurnCommand : public frc2::CommandHelper<frc2::CommandBase, TurnCommand> {
 public:
  TurnCommand(RobotContainer& container, units::degree_t target_angle,
              bool is_limelight_aim = false);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  DrivetrainSubsystem& drivetrain_;
  OptionalLimelightSubsystem& limelight_;

  units::degree_t target_angle_;
  bool is_done_ = false;
  bool is_limelight_aim_;
};

#endif  // ROBOT_COMMANDS_TURN_COMMAND_H_s