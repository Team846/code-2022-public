#ifndef ROBOT_COMMANDS_SPINUP_COMMAND_H_
#define ROBOT_COMMANDS_SPINUP_COMMAND_H_

#include <frc2/command/CommandHelper.h>

#include "robot/subsystems/robot_container.h"
#include "robot/subsystems/shooter.h"

// Spins up the shooter to a preset speed.
class SpinupCommand
    : public frc2::CommandHelper<frc2::CommandBase, SpinupCommand> {
 public:
  SpinupCommand(RobotContainer& container,
                const frc846::Pref<units::revolutions_per_minute_t>& speed);

  void Initialize() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  OptionalShooterSubsystem& shooter_;

  const frc846::Pref<units::revolutions_per_minute_t>& speed_;
};

#endif  // ROBOT_COMMANDS_SPINUP_COMMAND_H_