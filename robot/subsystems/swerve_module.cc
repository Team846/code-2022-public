#include "robot/subsystems/swerve_module.h"

#include <thread>

SwerveModuleSubsystem::SwerveModuleSubsystem(
    const frc846::Named& drivetrain, std::string location,
    units::degree_t fallback_cancoder_offset,
    frc846::motor::TalonFXConfigHelper* drive_esc_config_helper,
    frc846::motor::GainsHelper* drive_esc_gains_helper,
    frc846::motor::TalonFXConfigHelper* steer_esc_config_helper,
    frc846::motor::GainsHelper* steer_esc_gains_helper,
    frc846::motor::Converter<units::foot_t>& drive_converter,
    frc846::motor::Converter<units::degree_t>& steer_converter,
    int drive_esc_id, int steer_esc_id, int cancoder_id
  ) : frc846::Subsystem<SwerveModuleReadings, SwerveModuleTarget>{
          drivetrain,
          "module_" + location,
      },
      cancoder_offset_{*this, "cancoder_offset", fallback_cancoder_offset},
      
      drive_converter_(drive_converter),
      steer_converter_(steer_converter),
      drive_esc_{drive_esc_id},
      steer_esc_{steer_esc_id},
      drive_esc_helper_{*this, drive_esc_, drive_esc_config_helper, drive_esc_gains_helper},
      steer_esc_helper_{*this, steer_esc_, steer_esc_config_helper, steer_esc_gains_helper},
      cancoder_{cancoder_id} {
  drive_esc_helper_.OnInit([&] {
    drive_esc_.SetInverted(true);

    // Disable applied output frame
    drive_esc_helper_.DisableStatusFrames(
        {ctre::StatusFrameEnhanced::Status_1_General});
  });

  steer_esc_helper_.OnInit([&] {
    // Disable applied output frame
    steer_esc_helper_.DisableStatusFrames(
        {ctre::StatusFrameEnhanced::Status_1_General});
  });

  drive_esc_helper_.Setup();
  steer_esc_helper_.Setup();

  // Invert so that clockwise is positive when looking down on the robot
  cancoder_.ConfigSensorDirection(true);

  // Zero the module's direction with the CANcoder on start
  ZeroWithCANcoder();
}

std::pair<units::degree_t, bool> SwerveModuleSubsystem::NormalizedDirection(
    units::degree_t current, units::degree_t target) {
  // Normalize the `target` to be near the circle of `current`
  // `target` is in the domain of 0-360 while `current` can be anywhere

  // Find the nearest "0" to the current reading and add target to it
  units::degree_t normalized_target =
      current + (-units::math::fmod(current, 360_deg) + target);

  units::degree_t abs_err = units::math::abs(normalized_target - current);

  // Add or subtract 360 to the normalized target if 360 off
  if (abs_err > units::math::abs(normalized_target - 360_deg - current)) {
    normalized_target -= 360_deg;
  } else if (abs_err >
             units::math::abs(normalized_target + 360_deg - current)) {
    normalized_target += 360_deg;
  }

  // If the module has to turn more than 90 degrees, we can instead reverse the
  // drive motor and not have to rotate the module more than 90 degrees
  bool reverse_drive = false;
  if (normalized_target - current > 90_deg) {
    normalized_target = -(180_deg - normalized_target);
    reverse_drive = true;
  } else if (normalized_target - current < -90_deg) {
    normalized_target = normalized_target + 180_deg;
    reverse_drive = true;
  }

  return {normalized_target, reverse_drive};
}

void SwerveModuleSubsystem::ZeroWithCANcoder() {
  // Reads from the CANcoder to rezero the module's direction

  // Will attempt up to 5 times with 500ms delay between attempts if the
  // CANcoder returns an error (prevent trash absolute position reading bug)
  constexpr int kMaxAttempts = 5;
  constexpr int kSleepTimeMs = 500;

  units::degree_t module_direction = 0_deg;
  for (int attempts = 1; attempts <= kMaxAttempts; ++attempts) {
    Debug("CANCoder zero attempt {}/{}", attempts, kMaxAttempts);
    module_direction = units::degree_t(cancoder_.GetAbsolutePosition());

    if (cancoder_.GetLastError() == ctre::ErrorCode::OK) {
      Debug("Zeroed to {}!", module_direction);
      break;
    }

    Warn("Unable to zero", attempts, kMaxAttempts);

    if (attempts == kMaxAttempts) {
      Error("NOT ZEROED!!!");
    } else {
      Debug("Sleeping {}ms...", kSleepTimeMs);
      std::this_thread::sleep_for(std::chrono::milliseconds(kSleepTimeMs));
    }
  }

  zero_offset_ = module_direction - cancoder_offset_.value() -
                 steer_converter_.NativeToRealPosition(
                     steer_esc_.GetSelectedSensorPosition());

  last_direction_ = module_direction;
}

SwerveModuleTarget SwerveModuleSubsystem::ZeroTarget() const {
  SwerveModuleTarget target;
  target.speed = 0_fps;
  target.direction = 0_deg;
  return target;
}

bool SwerveModuleSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(drive_esc_helper_.VerifyConnected(), ok,
                "drive esc not connected");
  FRC846_VERIFY(drive_esc_.GetInverted() == true, ok,
                "drive esc incorrect invert state");
  FRC846_VERIFY(steer_esc_helper_.VerifyConnected(), ok,
                "steer esc not connected");
  FRC846_VERIFY(steer_esc_.GetInverted() == false, ok,
                "steer esc incorrect invert state");
  return ok;
}

SwerveModuleReadings SwerveModuleSubsystem::GetNewReadings() {
  SwerveModuleReadings readings;

  readings.speed = drive_converter_.NativeToRealVelocity(
      drive_esc_.GetSelectedSensorVelocity());
  readings.direction = steer_converter_.NativeToRealPosition(
                           steer_esc_.GetSelectedSensorPosition()) +
                       zero_offset_;
  readings.distance = drive_converter_.NativeToRealPosition(
      drive_esc_.GetSelectedSensorPosition());

  return readings;
}

void SwerveModuleSubsystem::WriteToHardware(SwerveModuleTarget target) {
  target_speed_graph_.Graph(target.speed);
  target_direction_graph_.Graph(target.direction);
  direction_graph_.Graph(steer_converter_.NativeToRealPosition(
      steer_esc_.GetSelectedSensorPosition()));
  cancoder_graph_.Graph(cancoder_.GetAbsolutePosition() * 1_deg);

  auto [normalized_angle, reverse_drive] =
      NormalizedDirection(readings().direction, target.direction);

  // Maintain steer position if not driving
  if (target.speed == 0_fps) {
    normalized_angle = last_direction_;
  } else {
    last_direction_ = normalized_angle;
  }

  double drive_output = drive_converter_.RealToNativeVelocity(
      target.speed * (reverse_drive ? -1 : 1));
  double steer_output =
      steer_converter_.RealToNativePosition(normalized_angle - zero_offset_);

  drive_esc_helper_.Write({frc846::motor::ControlMode::Velocity, drive_output});
  steer_esc_helper_.Write({frc846::motor::ControlMode::Position, steer_output});
}