#ifndef ROBOT_COMMANDS_AUTO_SHOOT_COMMAND_H_
#define ROBOT_COMMANDS_AUTO_SHOOT_COMMAND_H_

#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/trajectory_generator.h"
#include "robot/subsystems/robot_container.h"

// Waits for the shooter to get up to speed then runs the feeder to shoot.
class AutoShootCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoShootCommand> {
 public:
  AutoShootCommand(RobotContainer& container, units::second_t shoot_time,
                   units::second_t turn_timeout = 1_s);
};

#endif  // ROBOT_COMMANDS_AUTO_SHOOT_COMMAND_H_