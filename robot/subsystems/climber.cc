#include "robot/subsystems/climber.h"

#include <frc/RobotController.h>
#include <units/math.h>

ClimberSubsystem::ClimberSubsystem()
    : frc846::Subsystem<ClimberReadings, ClimberTarget>{"climber"} {}

void ClimberSubsystem::Zero() {
  left_arm_.Zero();
  right_arm_.Zero();
}

ClimberTarget ClimberSubsystem::ZeroTarget() const {
  ClimberTarget target;
  target.left_speed = 0.0;
  target.right_speed = 0.0;
  return target;
}

bool ClimberSubsystem::VerifyHardware() {
  bool ok = true;

  bool left_ok = left_arm_.VerifyHardware();
  if (!left_ok) {
    left_arm_.Error("Failed hardware verification!!");
  }
  ok = ok && left_ok;

  bool right_ok = right_arm_.VerifyHardware();
  if (!right_ok) {
    right_arm_.Error("Failed hardware verification!!");
  }
  ok = ok && right_ok;

  return ok;
}

ClimberReadings ClimberSubsystem::GetNewReadings() {
  ClimberReadings readings;

  left_arm_.UpdateReadings();
  right_arm_.UpdateReadings();

  readings.left = left_arm_.readings();
  readings.right = right_arm_.readings();

  return readings;
}

void ClimberSubsystem::WriteToHardware(ClimberTarget target) {
  left_arm_.SetTarget({target.left_speed});
  right_arm_.SetTarget({target.right_speed});

  left_arm_.UpdateHardware();
  right_arm_.UpdateHardware();
}
