#ifndef FRC846_MOTOR_GAINS_H_
#define FRC846_MOTOR_GAINS_H_

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/CANSparkMax.h>

#include "frc846/ctre_namespace.h"
#include "frc846/pref.h"
#include "units/time.h"

FRC846_CTRE_NAMESPACE()

namespace frc846::motor {

// Offloaded PID controller gains.
struct Gains {
  double p;
  double i;
  double d;
  double f;
  double max_integral_accumulator;
};

// PID controller gains helper for speed controllers.
//
// Creates preferences and write to speed controller.
class GainsHelper : public Named {
 public:
  GainsHelper(Named& parent, Gains gains);

  frc846::Pref<double> p_;
  frc846::Pref<double> i_;
  frc846::Pref<double> d_;
  frc846::Pref<double> f_;
  frc846::Pref<double> max_integral_accumulator_;

  // Write configuration to Talon SRX/FX speed controller.
  void Write(ctre::BaseTalon& esc, Gains& cache,
             units::time::millisecond_t timeout, bool ignore_cache = false);

  // Write configuration to Spark MAX speed controller.
  void Write(rev::SparkMaxPIDController& pid_controller, Gains& cache,
             bool ignore_cache = false);

 private:
  static constexpr int kTalonSlotIdx = 0;
};

}  // namespace frc846::motor

#endif  // FRC846_MOTOR_GAINS_H_