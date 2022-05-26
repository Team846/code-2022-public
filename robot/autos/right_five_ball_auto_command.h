#ifndef ROBOT_AUTOS_RIGHT_FIVE_BALL_AUTO_COMMAND_H_
#define ROBOT_AUTOS_RIGHT_FIVE_BALL_AUTO_COMMAND_H_

#include <frc2/command/SequentialCommandGroup.h>

#include "robot/field.h"
#include "robot/subsystems/robot_container.h"

class RightFiveBallAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 RightFiveBallAutoCommand> {
 public:
  RightFiveBallAutoCommand(RobotContainer& container);
};

#endif  // ROBOT_AUTOS_FRIGHT_FIVE_BALL_AUTO_COMMAND_H_