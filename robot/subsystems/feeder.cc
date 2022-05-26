#include "robot/subsystems/feeder.h"

FeederSubsystem::FeederSubsystem()
    : frc846::Subsystem<FeederReadings, FeederTarget>{"feeder"} {
  esc_helper_.OnInit([&] {
    esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

    // Disable all frames
    esc_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0,
         rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1,
         rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2});
  });

  esc_helper_.Setup();

  color_sensor_.ConfigureColorSensor(
      rev::ColorSensorV3::ColorResolution::k18bit,
      rev::ColorSensorV3::ColorMeasurementRate::k25ms);
  color_sensor_.ConfigureProximitySensor(
      rev::ColorSensorV3::ProximityResolution::k11bit,
      rev::ColorSensorV3::ProximityMeasurementRate::k6ms);
  color_sensor_.ConfigureProximitySensorLED(
      rev::ColorSensorV3::LEDPulseFrequency::k60kHz,
      rev::ColorSensorV3::LEDCurrent::kPulse10mA, 8);
}

FeederTarget FeederSubsystem::ZeroTarget() const {
  FeederTarget target;
  target.speed = 0.0;
  return target;
}

bool FeederSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(esc_helper_.VerifyConnected(), ok, "esc not connected");
  FRC846_VERIFY(esc_.GetInverted() == false, ok, "esc incorrect invert state");
  FRC846_VERIFY(color_sensor_.IsConnected(), ok, "color sensor not connected");
  return ok;
}

FeederReadings FeederSubsystem::GetNewReadings() {
  FeederReadings readings;

  readings.alliance = frc::DriverStation::GetAlliance();

  auto color = color_sensor_.GetColor();
  auto ir = color_sensor_.GetIR();

  readings.ball_state = FeederBallState::kNone;
  if (ir > ir_threshold_.value()) {
    if (color.blue >= blue_threshold_.value() && color.blue >= color.red) {
      readings.ball_state = FeederBallState::kBlue;
    } else if (color.red >= red_threshold_.value() && color.red >= color.blue) {
      readings.ball_state = FeederBallState::kRed;
    }
  }

  red_graph_.Graph(color.red);
  blue_graph_.Graph(color.blue);
  ir_graph_.Graph(ir);

  readings.is_running = is_running_;

  return readings;
}

void FeederSubsystem::WriteToHardware(FeederTarget target) {
  target_speed_graph_.Graph(target.speed);
  is_running_ = (target.speed == shoot_feed_speed_.value());
  esc_helper_.Write({frc846::motor::ControlMode::Percent, target.speed});
}