#ifndef ROBOT_SUBSYSTEMS_SWERVE_MODULE_H_
#define ROBOT_SUBSYSTEMS_SWERVE_MODULE_H_

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/sensors/CANCoder.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include <string>

#include "frc846/ctre_namespace.h"
#include "frc846/grapher.h"
#include "frc846/math.h"
#include "frc846/motor/converter.h"
#include "frc846/motor/helper.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"

FRC846_CTRE_NAMESPACE()

struct SwerveModuleReadings {
  units::feet_per_second_t speed;
  units::degree_t direction;
  units::foot_t distance;  // distance the wheel traveled on ground since start
};

struct SwerveModuleTarget {
  units::feet_per_second_t speed;
  units::degree_t direction;
};

class SwerveModuleSubsystem
    : public frc846::Subsystem<SwerveModuleReadings, SwerveModuleTarget> {
 public:
  SwerveModuleSubsystem(
      const frc846::Named& drivetrain, std::string location,
      units::degree_t fallback_cancoder_offset,
      frc846::motor::TalonFXConfigHelper* drive_esc_config_helper,
      frc846::motor::GainsHelper* drive_esc_gains_helper,
      frc846::motor::TalonFXConfigHelper* steer_esc_config_helper,
      frc846::motor::GainsHelper* steer_esc_gains_helper,
      frc846::motor::Converter<units::foot_t>& drive_converter,
      frc846::motor::Converter<units::degree_t>& steer_converter,
      int drive_esc_id, int steer_esc_id, int cancoder_id);

  // Calculate the normalized target angle for the module to minimize rotations.
  // Returns the normalized direction and whether or not the drive motor should
  // be reversed.
  static std::pair<units::degree_t, bool> NormalizedDirection(
      units::degree_t current, units::degree_t target);

  // Zero the module positions with the CANcoder.
  void ZeroWithCANcoder();

  SwerveModuleTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  // CANcoder magnet offset.
  frc846::Pref<units::degree_t> cancoder_offset_;

  // Magic value to offset steer position by.
  //
  // Initial CANcoder value - CANcoder magnet offset - Initial TalonFX relative
  // encoder value.
  units::degree_t zero_offset_;

  // The last direction the module was facing.
  units::degree_t last_direction_;

  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<units::feet_per_second_t> target_speed_graph_{target_named_,
                                                                "speed"};
  frc846::Grapher<units::degree_t> target_direction_graph_{target_named_,
                                                           "direction"};
  frc846::Grapher<units::degree_t> cancoder_graph_{*this, "cancoder"};
  frc846::Grapher<units::degree_t> direction_graph_{*this, "direction"};

  frc846::motor::Converter<units::foot_t>& drive_converter_;
  frc846::motor::Converter<units::degree_t>& steer_converter_;

  ctre::TalonFX drive_esc_;
  ctre::TalonFX steer_esc_;

  frc846::motor::TalonFXHelper drive_esc_helper_;
  frc846::motor::TalonFXHelper steer_esc_helper_;

  ctre::CANCoder cancoder_;

  SwerveModuleReadings GetNewReadings() override;

  void WriteToHardware(SwerveModuleTarget target) override;
};

#endif  // ROBOT_SUBSYSTEMS_SWERVE_MODULE_H_
