#ifndef ROBOT_COMMANDS_LEDS_COMMAND_H_
#define ROBOT_COMMANDS_LEDS_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "robot/subsystems/driver.h"
#include "robot/subsystems/limelight.h"
#include "robot/subsystems/robot_container.h"

// Sets the LED status lights.
//
// This command should constantly be running during teleop.
class LEDsCommand : public frc2::CommandHelper<frc2::CommandBase, LEDsCommand> {
 public:
  LEDsCommand(RobotContainer& container);

  void Execute() override;

 private:
  OptionalLEDsSubsystem& leds_;
  OptionalLimelightSubsystem& limelight_;
  DriverSubsystem& driver_;
};

#endif  // ROBOT_COMMANDS_LEDS_COMMAND_H_