#include "robot/subsystems/climber_arm.h"

#include <frc/RobotController.h>
#include <units/math.h>

#include "frc846/subsystem.h"

ClimberArmSubsystem::ClimberArmSubsystem(
    const frc846::Named& climber, std::string location, bool inverted,
    frc846::motor::SparkMAXConfigHelper* esc_config_helper, int leader_can_id,
    int follower_can_id)
    : frc846::Subsystem<ClimberArmReadings, ClimberArmTarget>{climber,
                                                              location},
      inverted_(inverted),
      leader_esc_{leader_can_id, rev::CANSparkMax::MotorType::kBrushless},
      follower_esc_{follower_can_id, rev::CANSparkMax::MotorType::kBrushless},
      leader_esc_helper_(*this, leader_esc_, esc_config_helper, nullptr),
      follower_esc_helper_(*this, follower_esc_, esc_config_helper, nullptr) {
  leader_esc_helper_.OnInit([&] {
    leader_esc_.SetInverted(inverted_);
    leader_esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Disable velocity frames
    leader_esc_helper_.DisableStatusFrames({
        rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1,
    });
  });

  follower_esc_helper_.OnInit([&] {
    follower_esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    frc846::motor::CheckOk(*this, follower_esc_.Follow(leader_esc_, false),
                           "follow");

    // Disable all frames
    follower_esc_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0,
         rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1,
         rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2});
  });

  leader_esc_helper_.Setup();
  follower_esc_helper_.Setup();

  Zero();
}

void ClimberArmSubsystem::Zero() {
  Debug("Zeroed leader encoder");
  leader_esc_helper_.encoder().SetPosition(0.0);
}

ClimberArmTarget ClimberArmSubsystem::ZeroTarget() const { return {0.0}; }

bool ClimberArmSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(leader_esc_helper_.VerifyConnected(), ok,
                "leader esc not connected");
  FRC846_VERIFY(follower_esc_helper_.VerifyConnected(), ok,
                "follower esc not connected");
  FRC846_VERIFY(follower_esc_.IsFollower(), ok, "not set as follower");
  return ok;
}

ClimberArmReadings ClimberArmSubsystem::GetNewReadings() {
  ClimberArmReadings readings;

  readings.position = arm_converter_.NativeToRealPosition(
      leader_esc_helper_.encoder().GetPosition());

  position_graph_.Graph(readings.position);

  return readings;
}

void ClimberArmSubsystem::WriteToHardware(ClimberArmTarget target) {
  target_speed_graph_.Graph(target.speed);

  // Write to both leader/follower to update configs
  leader_esc_helper_.Write({frc846::motor::ControlMode::Percent, target.speed});
  follower_esc_helper_.Write(
      {frc846::motor::ControlMode::Percent, target.speed});
}