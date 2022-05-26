#ifndef ROBOT_COMMANDS_SMART_SHOOT_COMMAND_H_
#define ROBOT_COMMANDS_SMART_SHOOT_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "robot/subsystems/feeder.h"
#include "robot/subsystems/robot_container.h"

// Runs the feeder to shoot balls, checking if they're the right color. If the
// ball is the wrong color, it'll stop shooting until the shooter slows down.
//
// Smart shoot runs the feeder much slower since it has to check the color
// sensor on every tick. If the operator knows both balls are the correct color,
// run "dumb" shoot instead (`feeder_command`) at a higher speed.
class SmartShootCommand
    : public frc2::CommandHelper<frc2::CommandBase, SmartShootCommand> {
 public:
  SmartShootCommand(RobotContainer& container);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalFeederSubsystem& feeder_;
};

#endif  // ROBOT_COMMANDS_SMART_SHOOT_COMMAND_H_