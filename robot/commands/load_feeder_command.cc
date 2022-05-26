#include "robot/commands/load_feeder_command.h"

#include "robot/subsystems/feeder.h"

LoadFeederCommand::LoadFeederCommand(RobotContainer& container,
                                     const frc846::Pref<bool>& should_run,
                                     bool is_auto)
    : feeder_(container.feeder_),
      shooter_(container.shooter_),
      should_run_(should_run),
      is_auto_(is_auto) {
  AddRequirements({&feeder_});
  SetName("load_feeder_command");
}

void LoadFeederCommand::Execute() {
  if (feeder_.Initialized()) {
    auto alliance_color = feeder_.readings().alliance;
    auto ball_state = feeder_.subsystem()->readings().ball_state;

    // Check for correct ball
    auto is_wrong_ball =
        ((ball_state == FeederBallState::kBlue &&
          alliance_color == frc::DriverStation::Alliance::kRed) ||
         (ball_state == FeederBallState::kRed &&
          alliance_color == frc::DriverStation::Alliance::kBlue) ||
         (ball_state == FeederBallState::kNone));

    // Check if shooter speed is at eject speed
    auto can_eject = shooter_.subsystem()->readings().speed <=
                     shooter_.subsystem()->load_speed_threshold_.value();

    // Check if you can keep loading
    auto should_load =
        should_run_.value() && is_wrong_ball &&
        (can_eject || is_auto_ || ball_state == FeederBallState::kNone);

    // Set feeder target speed
    if (should_load) {
      feeder_.SetTarget({feeder_.subsystem()->load_feed_speed_.value()});
    } else {
      feeder_.SetTargetZero();
    }
  }
}

void LoadFeederCommand::End(bool interrupted) {
  (void)interrupted;
  if (feeder_.Initialized()) {
    feeder_.SetTargetZero();
  }
}

bool LoadFeederCommand::IsFinished() { return false; }