#ifndef ROBOT_COMMANDS_INTAKE_COMMAND_H_
#define ROBOT_COMMANDS_INTAKE_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "robot/subsystems/intake.h"
#include "robot/subsystems/robot_container.h"

// Puts out the intake and runs the rollers.
class IntakeCommand
    : public frc2::CommandHelper<frc2::CommandBase, IntakeCommand> {
 public:
  IntakeCommand(RobotContainer& container, bool reverse = false);

  void Initialize() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalIntakeSubsystem& intake_;

  bool reverse_;
};

#endif  // ROBOT_COMMANDS_INTAKE_COMMAND_H_