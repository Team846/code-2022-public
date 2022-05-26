#ifndef ROBOT_COMMANDS_CLIMBER_SPIN_COMMAND_H_
#define ROBOT_COMMANDS_CLIMBER_SPIN_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "frc846/subsystem.h"
#include "robot/subsystems/robot_container.h"

// Spins the climber arms at a specified speed.
class ClimberSpinCommand
    : public frc2::CommandHelper<frc2::CommandBase, ClimberSpinCommand> {
 public:
  ClimberSpinCommand(RobotContainer& container,
                     const frc846::Pref<double>& target);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalClimberSubsystem& climber_;

  const frc846::Pref<double>& target_;
};

#endif  // ROBOT_COMMANDS_CLIMBER_SPIN_COMMAND_H_