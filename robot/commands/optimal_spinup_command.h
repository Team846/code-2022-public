#ifndef ROBOT_COMMANDS_OPTIMAL_SPINUP_COMMAND_H_
#define ROBOT_COMMANDS_OPTIMAL_SPINUP_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "robot/subsystems/drivetrain.h"
#include "robot/subsystems/limelight.h"
#include "robot/subsystems/robot_container.h"
#include "robot/subsystems/shooter.h"

// Spins up the shooter to the optimal speed based on limelight data.
class OptimalSpinupCommand
    : public frc2::CommandHelper<frc2::CommandBase, OptimalSpinupCommand>,
      frc846::Named {
 public:
  OptimalSpinupCommand(RobotContainer& container);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalShooterSubsystem& shooter_;
  OptionalLimelightSubsystem& limelight_;
  DrivetrainSubsystem& drivetrain_;

  units::revolutions_per_minute_t target_;
};

#endif  // ROBOT_COMMANDS_OPTIMAL_SPINUP_COMMAND_H_