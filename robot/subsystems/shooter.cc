#include "robot/subsystems/shooter.h"

#include "robot/field.h"

ShooterSubsystem::ShooterSubsystem()
    : frc846::Subsystem<ShooterReadings, ShooterTarget>{"shooter"} {
  left_esc_helper_.OnInit([&] {
    // Disable applied output/position frames
    left_esc_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0,
         rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2});
  });

  right_esc_helper_.OnInit([&] {
    right_esc_.SetInverted(true);

    // Disable applied output/position frames
    right_esc_helper_.DisableStatusFrames(
        {rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0,
         rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2});
  });

  left_esc_helper_.Setup();
  right_esc_helper_.Setup();
}

units::second_t ShooterSubsystem::GetTimeFlight(units::degree_t ty,
                                                units::foot_t distance) {
  units::feet_per_second_t exit_velocity =
      flywheel_radius_.value() * GetOptimalRPM(ty) *
      units::math::cos(shooter_angle_.value()) / 1_rad;

  return distance / exit_velocity;
}

units::revolutions_per_minute_t ShooterSubsystem::GetOptimalRPM(
    units::degree_t target_vertical_angle) {
  double a_term =
      a_.value() * units::math::pow<2>(target_vertical_angle).to<double>();
  double b_term = b_.value() * target_vertical_angle.to<double>();
  double c_term = c_.value();

  return units::revolutions_per_minute_t(a_term + b_term + c_term);
}

ShooterTarget ShooterSubsystem::ZeroTarget() const {
  ShooterTarget target;
  // Default to eject speed
  target.speed = eject_speed_.value();
  return target;
}

bool ShooterSubsystem::VerifyHardware() {
  bool ok = true;
  FRC846_VERIFY(left_esc_helper_.VerifyConnected(), ok,
                "left esc not connected");
  FRC846_VERIFY(left_esc_.GetInverted() == false, ok,
                "left esc incorrect invert state");
  FRC846_VERIFY(right_esc_helper_.VerifyConnected(), ok,
                "right esc not connected");
  FRC846_VERIFY(right_esc_.GetInverted() == true, ok,
                "right esc incorrect invert state");
  return ok;
}

units::degree_t ShooterSubsystem::DistanceToTy(units::foot_t distance,
                                               units::degree_t mounting_angle,
                                               units::foot_t mounting_height) {
  units::foot_t vertical_distance = field::hub::kTopHeight - mounting_height;
  return units::math::atan((vertical_distance) /
                           (distance - (field::hub::kOuterDiameter / 2))) -
         mounting_angle;
}

std::pair<units::revolutions_per_minute_t, frc846::Vector2D<units::foot_t>>
ShooterSubsystem::GetCompensatedRPM(
    frc846::Vector2D<units::feet_per_second_t> drivetrain_velocity,
    LimelightReadings limelight, units::degree_t mounting_angle,
    units::foot_t mounting_height) {
  units::foot_t hub_distance = limelight.hub_distance;
  frc846::Vector2D<units::foot_t> hub_pos{
      hub_distance * units::math::sin(limelight.tx),
      hub_distance * units::math::cos(limelight.tx),
  };

  units::second_t time_of_flight = GetTimeFlight(limelight.ty, hub_distance);
  frc846::Vector2D<units::foot_t> hub_pos_compensation{
      drivetrain_velocity.x * time_of_flight,
      drivetrain_velocity.y * time_of_flight,
  };

  frc846::Vector2D<units::foot_t> compensated_hub_pos =
      hub_pos + hub_pos_compensation;
  units::foot_t compensated_hub_distance = compensated_hub_pos.Magnitude();

  units::degree_t final_ty =
      DistanceToTy(compensated_hub_distance, mounting_angle, mounting_height);

  return {
      GetOptimalRPM(final_ty) - GetOptimalRPM(limelight.ty),
      compensated_hub_pos,
  };
}

ShooterReadings ShooterSubsystem::GetNewReadings() {
  auto left_rpm =
      converter_.NativeToRealVelocity(left_esc_helper_.encoder().GetVelocity());
  auto right_rpm = converter_.NativeToRealVelocity(
      right_esc_helper_.encoder().GetVelocity());

  left_rpm_graph_.Graph(left_rpm);
  right_rpm_graph_.Graph(right_rpm);
  rpm_difference_graph_.Graph(left_rpm - right_rpm);

  ShooterReadings readings;
  readings.speed = (left_rpm + right_rpm) / 2;

  readings.is_ready =
      units::math::abs(target_speed_ - readings.speed) < tolerance_.value() &&
      target_speed_ >= preset_close_.value();
  return readings;
}

void ShooterSubsystem::WriteToHardware(ShooterTarget target) {
  target_speed_graph_.Graph(target.speed);
  target_speed_ = target.speed;
  auto native_rpm = converter_.RealToNativeVelocity(target.speed);

  left_esc_helper_.Write({frc846::motor::ControlMode::Velocity,
                          native_rpm * left_bonus_factor_.value()});
  right_esc_helper_.Write({frc846::motor::ControlMode::Velocity, native_rpm});
}