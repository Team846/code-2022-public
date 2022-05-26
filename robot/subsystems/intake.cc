#include "robot/subsystems/intake.h"

IntakeSubsystem::IntakeSubsystem()
    : frc846::Subsystem<IntakeReadings, IntakeTarget>{"intake"} {
  esc_helper_.OnInit([&] {
    esc_.SetInverted(true);

    // Disable all frames
    esc_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0,
         rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1,
         rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2});
  });

  esc_helper_.Setup();
}

IntakeTarget IntakeSubsystem::ZeroTarget() const {
  IntakeTarget target;
  target.is_extended = false;
  target.speed = 0;
  return target;
}

bool IntakeSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_helper_.VerifyConnected(), ok, "esc not connected");
  FRC846_VERIFY(esc_.GetInverted() == true, ok, "esc incorrect invert state");
  return ok;
}

IntakeReadings IntakeSubsystem::GetNewReadings() { return {}; }

void IntakeSubsystem::WriteToHardware(IntakeTarget target) {
  target_is_extended_graph_.Graph(target.is_extended);
  target_speed_graph_.Graph(target.speed);

  intake_solenoid_.Set(target.is_extended);

  esc_helper_.Write({frc846::motor::ControlMode::Percent, target.speed});
}
