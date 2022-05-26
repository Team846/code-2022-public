#ifndef ROBOT_COMMANDS_LOAD_FEEDER_COMMAND_H_
#define ROBOT_COMMANDS_LOAD_FEEDER_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "robot/subsystems/feeder.h"
#include "robot/subsystems/robot_container.h"
#include "robot/subsystems/shooter.h"

// Loads the feeder with a ball, ejecting it if it's the wrong color.
//
// This command should constantly be running during teleop.
class LoadFeederCommand
    : public frc2::CommandHelper<frc2::CommandBase, LoadFeederCommand> {
 public:
  LoadFeederCommand(RobotContainer& container,
                    const frc846::Pref<bool>& should_run, bool is_auto = false);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalFeederSubsystem& feeder_;
  OptionalShooterSubsystem& shooter_;

  const frc846::Pref<bool>& should_run_;

  bool is_auto_;
};

#endif  // ROBOT_COMMANDS_LOAD_FEEDER_COMMAND_H_