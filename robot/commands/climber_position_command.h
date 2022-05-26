#ifndef ROBOT_COMMANDS_CLIMB_POSITION_COMMAND_H_
#define ROBOT_COMMANDS_CLIMB_POSITION_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "robot/subsystems/robot_container.h"

// Moves the climber arms to a certain setpoint position. `allowed_direction`
// can be set to restrict which direction the arms can travel.
class ClimberPositionCommand
    : public frc2::CommandHelper<frc2::CommandBase, ClimberPositionCommand> {
 public:
  enum Direction { kClimb, kSwing, kEither };

  ClimberPositionCommand(RobotContainer& container,
                         const ClimberSetpointPref& setpoint,
                         Direction allowed_direction);

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalClimberSubsystem& climber_;

  const ClimberSetpointPref& setpoint_;

  Direction allowed_direction_;
  bool is_done_ = false;
};

#endif  // ROBOT_COMMANDS_CLIMB_POSITION_COMMAND_H_