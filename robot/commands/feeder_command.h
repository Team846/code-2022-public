#ifndef ROBOT_COMMANDS_FEEDER_COMMAND_H_
#define ROBOT_COMMANDS_FEEDER_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "robot/subsystems/feeder.h"
#include "robot/subsystems/robot_container.h"

// Runs the feeder at a specified speed. Used for dumb shoot or reversing the
// feeder.
class FeederCommand
    : public frc2::CommandHelper<frc2::CommandBase, FeederCommand> {
 public:
  FeederCommand(RobotContainer& container, const frc846::Pref<double>& speed);

  void Initialize() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalFeederSubsystem& feeder_;

  const frc846::Pref<double>& speed_;
};

#endif  // ROBOT_COMMANDS_FEEDER_COMMAND_H_