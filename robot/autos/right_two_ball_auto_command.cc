#include "robot/autos/right_two_ball_auto_command.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelDeadlineGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/WaitCommand.h>

#include "robot/commands/auto_drive_and_intake_command.h"
#include "robot/commands/auto_shoot_command.h"
#include "robot/commands/climber_position_command.h"
#include "robot/commands/intake_command.h"
#include "robot/commands/optimal_spinup_command.h"

RightTwoBallAutoCommand::RightTwoBallAutoCommand(RobotContainer& container) {
  SetName("right_two_ball_auto_command");

  // Set starting position.
  AddCommands(frc2::InstantCommand{[&] {
    container.drivetrain_.SetPoint(field::points::kRightStart().point);
    container.drivetrain_.SetBearing(field::points::kRightStart().bearing);
  }});

  // Start running the flywheel in the background.
  AddCommands(frc2::ScheduleCommand{new OptimalSpinupCommand{container}});

  // Shoot preload.
  AddCommands(frc2::ParallelCommandGroup{
      //   frc2::ParallelDeadlineGroup{
      AutoShootCommand{container, 1_s},
      //   IntakeCommand{container},
      //   },
      ClimberPositionCommand{
          container,
          container.climber_.subsystem()->setpoints_match_stow_,
          ClimberPositionCommand::Direction::kEither,
      },
  });

  //   AddCommands(preload);

  // Intake two right balls and shoot.
  AddCommands(
      AutoDriveAndIntakeCommand{
          container,
          {
              {field::points::kRightIntake1(), 0_fps},
          },
          field::points::kRightIntake1().point.Bearing(),
      },
      AutoShootCommand{container, 5_s});
}