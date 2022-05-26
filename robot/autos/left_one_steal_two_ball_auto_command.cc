#include "robot/autos/left_one_steal_two_ball_auto_command.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ProxyScheduleCommand.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/WaitCommand.h>

#include "robot/commands/auto_drive_and_intake_command.h"
#include "robot/commands/auto_shoot_command.h"
#include "robot/commands/climber_position_command.h"
#include "robot/commands/feeder_command.h"
#include "robot/commands/optimal_spinup_command.h"
#include "robot/commands/spinup_command.h"
#include "robot/field.h"

LeftOneStealTwoBallAutoCommand::LeftOneStealTwoBallAutoCommand(
    RobotContainer& container) {
  SetName("LeftOneStealTwoBallAutoCommand");

  // Set starting position.
  AddCommands(frc2::InstantCommand{[&] {
    container.drivetrain_.SetPoint(field::points::kLeftStart().point);
    container.drivetrain_.SetBearing(field::points::kLeftStart().bearing);
  }});

  // Start running the flywheel in the background.
  AddCommands(frc2::ScheduleCommand{new OptimalSpinupCommand{container}});

  if (container.climber_.Initialized()) {
    AddCommands(ClimberPositionCommand{
        container,
        container.climber_.subsystem()->setpoints_match_stow_,
        ClimberPositionCommand::Direction::kEither,
    });
  }

  // Intake left ball, drive back to tarmac line, and shoot.
  AddCommands(
      AutoDriveAndIntakeCommand{
          container,
          {
              {field::points::kLeftIntake1(), 0_fps},
          },
          field::points::kLeftIntake1().point.Bearing(),
      },
      frc2::WaitCommand(0.5_s), AutoShootCommand{container, 3_s});

  AddCommands(AutoDriveAndIntakeCommand{
      container,
      {
          {field::points::kLeftSteal1(), 0_fps},
          {{{-9.53_ft, -8.20_ft - 2_ft}, 0_deg}, 0_fps},
      },
      0_deg,
      false, /* limelight_aim */
  });
}