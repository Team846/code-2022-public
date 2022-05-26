#ifndef ROBOT_AUTOS_LEFT_TWO_BALL_AUTO_COMMAND_H_2
#define ROBOT_AUTOS_LEFT_TWO_BALL_AUTO_COMMAND_H_2

#include <frc2/command/SequentialCommandGroup.h>

#include "robot/subsystems/robot_container.h"

class LeftOneStealTwoBallAutoCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 LeftOneStealTwoBallAutoCommand> {
 public:
  LeftOneStealTwoBallAutoCommand(RobotContainer& container);
};

#endif  // ROBOT_AUTOS_LEFT_TWO_BALL_AUTO_COMMAND_H_2