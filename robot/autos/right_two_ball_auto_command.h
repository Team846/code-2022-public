#ifndef ROBOT_AUTOS_RIGHT_TWO_BALL_AUTO_COMMAND_H_
#define ROBOT_AUTOS_RIGHT_TWO_BALL_AUTO_COMMAND_H_

#include <frc2/command/SequentialCommandGroup.h>

#include "robot/field.h"
#include "robot/subsystems/robot_container.h"

class RightTwoBallAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 RightTwoBallAutoCommand> {
 public:
  RightTwoBallAutoCommand(RobotContainer& container);
};

#endif  // ROBOT_AUTOS_FRIGHT_TWO_BALL_AUTO_COMMAND_H_