#include "robot/commands/auto_shoot_command.h"

#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitUntilCommand.h>

#include "robot/commands/feeder_command.h"

AutoShootCommand::AutoShootCommand(RobotContainer& container,
                                   units::second_t shoot_time,
                                   units::second_t turn_timeout) {
  SetName("auto_shoot_command");

  AddCommands(
      // Wait until shooter within speed
      frc2::WaitUntilCommand([&] {
        return container.shooter_.readings().is_ready;
      }).WithTimeout(turn_timeout),
      // Run feeder for some time
      FeederCommand{container, container.feeder_.subsystem()->shoot_feed_speed_}
          .WithTimeout(shoot_time));
}