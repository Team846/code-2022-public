#ifndef FRC846_MOTOR_CONFIG_H_
#define FRC846_MOTOR_CONFIG_H_

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/CANSparkMax.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

#include "frc846/ctre_namespace.h"
#include "frc846/pref.h"

FRC846_CTRE_NAMESPACE()

namespace frc846::motor {

// Base configurations for VictorSPX speed controllers.
struct VictorSPXConfig {
  double peak_output;  // [0, 1]
  units::volt_t voltage_comp_saturation;
};

// Base configurations for TalonSRX speed controllers.
struct TalonSRXConfig {
  double peak_output;  // [0, 1]
  units::volt_t voltage_comp_saturation;

  units::ampere_t peak_current_limit;
  units::millisecond_t peak_current_duration;
  units::ampere_t continuous_current_limit;
};

// Base configurations for TalonFX speed controllers.
struct TalonFXConfig {
  double peak_output;  // [0, 1]
  units::volt_t voltage_comp_saturation;

  units::ampere_t supply_peak_current_limit;
  units::millisecond_t supply_peak_current_duration;
  units::ampere_t supply_continuous_current_limit;

  units::ampere_t stator_peak_current_limit;
  units::millisecond_t stator_peak_current_duration;
  units::ampere_t stator_continuous_current_limit;
};

// Base configurations for Spark MAX speed controllers.
struct SparkMAXConfig {
  double peak_output;  // [0, 1]
  units::volt_t voltage_comp_saturation;

  units::ampere_t current_limit;
};

// Configuration helper for VictorSPX speed controllers.
//
// Creates preferences and write to speed controller.
class VictorSPXConfigHelper : public Named {
 public:
  VictorSPXConfigHelper(const Named& parent, VictorSPXConfig config);

  frc846::Pref<double> peak_output_;
  frc846::Pref<units::volt_t> voltage_comp_saturation_;

  // Write configuration to speed controller.
  void Write(ctre::VictorSPX& esc, VictorSPXConfig& cache,
             units::millisecond_t timeout, bool ignore_cache = false);
};

// Configuration helper for TalonSRX speed controllers.
//
// Creates preferences and write to speed controller.
class TalonSRXConfigHelper : public Named {
 public:
  TalonSRXConfigHelper(const Named& parent, TalonSRXConfig config);

  frc846::Pref<double> peak_output_;
  frc846::Pref<units::volt_t> voltage_comp_saturation_;

  frc846::Pref<units::ampere_t> peak_current_limit_;
  frc846::Pref<units::millisecond_t> peak_current_duration_;
  frc846::Pref<units::ampere_t> continuous_current_limit_;

  // Write configuration to speed controller.
  void Write(ctre::TalonSRX& esc, TalonSRXConfig& cache,
             units::millisecond_t timeout, bool ignore_cache = false);
};

// Configuration helper for TalonFX speed controllers.
//
// Creates preferences and write to speed controller.
class TalonFXConfigHelper : public Named {
 public:
  TalonFXConfigHelper(const Named& parent, TalonFXConfig config);

  frc846::Pref<double> peak_output_;
  frc846::Pref<units::volt_t> voltage_comp_saturation_;

  frc846::Pref<units::ampere_t> supply_peak_current_limit_;
  frc846::Pref<units::millisecond_t> supply_peak_current_duration_;
  frc846::Pref<units::ampere_t> supply_continuous_current_limit_;

  frc846::Pref<units::ampere_t> stator_peak_current_limit_;
  frc846::Pref<units::millisecond_t> stator_peak_current_duration_;
  frc846::Pref<units::ampere_t> stator_continuous_current_limit_;

  // Write configuration to speed controller.
  void Write(ctre::TalonFX& esc, TalonFXConfig& cache,
             units::millisecond_t timeout, bool ignore_cache = false);
};

// Configuration helper for SparkMAX speed controllers.
//
// Creates preferences and write to speed controller.
class SparkMAXConfigHelper : public Named {
 public:
  SparkMAXConfigHelper(const Named& parent, SparkMAXConfig config);

  frc846::Pref<double> peak_output_;
  frc846::Pref<units::volt_t> voltage_comp_saturation_;

  frc846::Pref<units::ampere_t> current_limit_;

  // Write configuration to speed controller.
  void Write(rev::CANSparkMax& esc, rev::SparkMaxPIDController& pid_controller,
             SparkMAXConfig& cache, bool ignore_cache = false);
};

}  // namespace frc846::motor

#endif  // FRC846_MOTOR_CONFIG_H_