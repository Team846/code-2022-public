#include "robot/commands/auto_drive_and_intake_command.h"

#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "robot/commands/follow_trajectory_command.h"
#include "robot/commands/intake_command.h"
#include "robot/commands/load_feeder_command.h"
#include "robot/commands/spinup_command.h"
#include "robot/commands/turn_command.h"
#include "robot/field.h"
#include "robot/subsystems/robot_container.h"

AutoDriveAndIntakeCommand::AutoDriveAndIntakeCommand(
    RobotContainer& container, std::vector<frc846::InputWaypoint> points,
    units::degree_t end_bearing, bool limelight_aim,
    units::second_t turn_timeout) {
  SetName("auto_drive_and_intake_command");

  AddCommands(frc2::ParallelDeadlineGroup{
      frc2::SequentialCommandGroup{
          FollowTrajectoryCommand{container, points},
          TurnCommand{container, end_bearing, limelight_aim}.WithTimeout(
              turn_timeout),
      },
      IntakeCommand{container},
      LoadFeederCommand{
          container,
          container.feeder_.subsystem()->enable_auto_load_,
          true,
      },
  });
}