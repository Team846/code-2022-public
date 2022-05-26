#ifndef ROBOT_AUTOS_LEFT_TWO_BALL_AUTO_COMMAND_H_
#define ROBOT_AUTOS_LEFT_TWO_BALL_AUTO_COMMAND_H_

#include <frc2/command/SequentialCommandGroup.h>

#include "robot/subsystems/robot_container.h"

class LeftTwoBallAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 LeftTwoBallAutoCommand> {
 public:
  LeftTwoBallAutoCommand(RobotContainer& container);
};

#endif  // ROBOT_AUTOS_LEFT_TWO_BALL_AUTO_COMMAND_H_