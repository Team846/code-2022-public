#include "robot/commands/smart_shoot_command.h"

SmartShootCommand::SmartShootCommand(RobotContainer& container)
    : feeder_(container.feeder_) {
  AddRequirements({&feeder_});
  SetName("smart_shoot_command");
}
void SmartShootCommand::Execute() {
  if (feeder_.Initialized()) {
    auto alliance_color = feeder_.readings().alliance;
    auto ball_state = feeder_.subsystem()->readings().ball_state;

    if ((ball_state == FeederBallState::kBlue &&
         alliance_color == frc::DriverStation::Alliance::kBlue) ||
        (ball_state == FeederBallState::kRed &&
         alliance_color == frc::DriverStation::Alliance::kRed) ||
        (ball_state == FeederBallState::kNone)) {
      feeder_.SetTarget({feeder_.subsystem()->shoot_feed_speed_.value()});
    } else {
      feeder_.SetTargetZero();
    }
  }
}

void SmartShootCommand::End(bool interrupted) {
  (void)interrupted;
  if (feeder_.Initialized()) {
    feeder_.SetTargetZero();
  }
}

bool SmartShootCommand::IsFinished() { return false; }