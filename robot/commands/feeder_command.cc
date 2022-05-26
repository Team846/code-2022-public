#include "robot/commands/feeder_command.h"

FeederCommand::FeederCommand(RobotContainer& container,
                             const frc846::Pref<double>& speed)
    : feeder_(container.feeder_), speed_(speed) {
  AddRequirements({&feeder_});
  SetName("feeder_command");
}

void FeederCommand::Initialize() {
  if (feeder_.Initialized()) {
    FeederTarget feeder_target;
    feeder_target.speed = speed_.value();

    feeder_.SetTarget(feeder_target);
  }
}

void FeederCommand::End(bool interrupted) {
  (void)interrupted;
  if (feeder_.Initialized()) {
    feeder_.SetTargetZero();
  }
}

bool FeederCommand::IsFinished() { return false; }