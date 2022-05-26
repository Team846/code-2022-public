#include "robot/commands/climber_spin_command.h"

#include "robot/subsystems/robot_container.h"

ClimberSpinCommand::ClimberSpinCommand(RobotContainer& container,
                                       const frc846::Pref<double>& target)
    : climber_(container.climber_), target_(target) {
  AddRequirements({&climber_});
  SetName("climber_spin_command");
}

void ClimberSpinCommand::Execute() {
  if (climber_.Initialized()) {
    climber_.SetTarget({
        target_.value(),
        target_.value(),
    });
  }
}

void ClimberSpinCommand::End(bool interrupted) {
  (void)interrupted;
  if (climber_.Initialized()) {
    climber_.SetTargetZero();
  }
}

bool ClimberSpinCommand::IsFinished() { return false; }