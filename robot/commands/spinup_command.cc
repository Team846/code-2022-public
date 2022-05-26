#include "robot/commands/spinup_command.h"

SpinupCommand::SpinupCommand(
    RobotContainer& container,
    const frc846::Pref<units::revolutions_per_minute_t>& speed)
    : shooter_(container.shooter_), speed_(speed) {
  AddRequirements({&shooter_});
  SetName("spinup_command");
}

void SpinupCommand::Initialize() {
  if (shooter_.Initialized()) {
    shooter_.SetTarget({speed_.value()});
  }
}

void SpinupCommand::End(bool interrupted) {
  (void)interrupted;
  if (shooter_.Initialized()) {
    shooter_.SetTargetZero();
  }
}

bool SpinupCommand::IsFinished() { return false; }