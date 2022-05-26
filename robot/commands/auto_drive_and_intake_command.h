#ifndef ROBOT_COMMANDS_AUTO_DRIVE_AND_INTAKE_H_
#define ROBOT_COMMANDS_AUTO_DRIVE_AND_INTAKE_H_

#include <frc2/command/SequentialCommandGroup.h>

#include "frc846/trajectory_generator.h"
#include "robot/subsystems/robot_container.h"

// Follows a trajectory with the intake and feeder running. At the end of the
// trajectory, also points to a certain direction and limelight aim if a target
// exists.
class AutoDriveAndIntakeCommand
    : public frc2::CommandHelper<frc2::SequentialCommandGroup,
                                 AutoDriveAndIntakeCommand> {
 public:
  AutoDriveAndIntakeCommand(RobotContainer& container,
                            std::vector<frc846::InputWaypoint> points,
                            units::degree_t end_bearing,
                            bool limelight_aim = true,
                            units::second_t turn_timeout = 1_s);
};

#endif  // ROBOT_COMMANDS_AUTO_DRIVE_AND_INTAKE_H_