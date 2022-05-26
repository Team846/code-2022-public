#ifndef ROBOT_COMMANDS_DRIVE_COMMAND_H_
#define ROBOT_COMMANDS_DRIVE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "robot/subsystems/driver.h"
#include "robot/subsystems/drivetrain.h"
#include "robot/subsystems/feeder.h"
#include "robot/subsystems/limelight.h"
#include "robot/subsystems/robot_container.h"
#include "robot/subsystems/shooter.h"

// Handles reading driver inputs to control the drivetrain (manual translation
// and steering, slow mode, robot centric control mode, vision aiming).
//
// This command should constantly be running during teleop.
class DriveCommand
    : public frc2::CommandHelper<frc2::CommandBase, DriveCommand> {
 public:
  DriveCommand(RobotContainer& container);

  void Execute() override;

  bool IsFinished() override;

 private:
  DriverSubsystem& driver_;
  OperatorSubsystem& operator_;
  DrivetrainSubsystem& drivetrain_;
  OptionalLimelightSubsystem& limelight_;
  OptionalShooterSubsystem& shooter_;
  OptionalFeederSubsystem& feeder_;
};

#endif  // ROBOT_COMMANDS_DRIVE_COMMAND_H_