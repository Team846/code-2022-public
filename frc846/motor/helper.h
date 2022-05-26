#ifndef FRC846_MOTOR_HELPER_H_
#define FRC846_MOTOR_HELPER_H_

#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>
#include <rev/CANSparkMax.h>

#include <initializer_list>

#include "frc846/ctre_namespace.h"
#include "frc846/motor/config.h"
#include "frc846/motor/gains.h"

FRC846_CTRE_NAMESPACE()

namespace frc846::motor {

// Default CAN timeout for speed controllers.
static constexpr units::millisecond_t kCANTimeout = 50_ms;

// Motor output control mode.
enum ControlMode { Percent, Velocity, Position, Current };

// Convert control mode to ctre control mode.
constexpr ctre::ControlMode CTREControlMode(ControlMode mode);

// Convert control mode to rev control type.
constexpr rev::CANSparkMax::ControlType RevControlMode(ControlMode mode);

// Check ctre status code ok.
void CheckOk(const Named& named, ctre::ErrorCode err, std::string_view field);

// Check rev status code ok.
void CheckOk(const Named& named, rev::REVLibError err, std::string_view field);

// Motor output.
struct Output {
  ControlMode mode;
  double value;

  bool operator==(const Output& other) {
    return mode == other.mode && value == other.value;
  }

  bool operator!=(const Output& other) { return !(*this == other); }
};

// Helper for VictorSPX speed controllers.
//
// Handles motor setup, configurations, outputting, and CAN status frame usage.
class VictorSPXHelper {
 public:
  VictorSPXHelper(const Named& parent, ctre::VictorSPX& esc,
                  VictorSPXConfigHelper* config);

  ~VictorSPXHelper();

  // Call setup routines and write initial configs and gains.
  void Setup(units::millisecond_t timeout = kCANTimeout);

  // Disable given status frames.
  void DisableStatusFrames(
      std::initializer_list<ctre::StatusFrameEnhanced> frames,
      units::millisecond_t timeout = kCANTimeout);

  // Verify that the speed controller connected.
  bool VerifyConnected();

  // Add a setup routine that's called on intialization and when a reset is
  // detected.
  void OnInit(std::function<void()> callback);

  // Write a new output to the speed controller.
  void Write(Output output, units::millisecond_t timeout = kCANTimeout);

 private:
  const Named& parent_;

  ctre::VictorSPX& esc_;

  std::vector<std::function<void()>> on_inits_;

  VictorSPXConfigHelper* config_;

  VictorSPXConfig config_cache_;
};

// Helper for TalonSRX speed controllers.
//
// Handles motor setup, configurations, gains, outputting, and CAN status frame
// usage.
class TalonSRXHelper {
 public:
  TalonSRXHelper(const Named& parent, ctre::TalonSRX& esc,
                 TalonSRXConfigHelper* config, GainsHelper* gains);

  ~TalonSRXHelper();

  // Call setup routines and write initial configs and gains.
  void Setup(units::millisecond_t timeout = kCANTimeout);

  // Disable given status frames.
  void DisableStatusFrames(
      std::initializer_list<ctre::StatusFrameEnhanced> frames,
      units::millisecond_t timeout = kCANTimeout);

  // Verify that the speed controller connected.
  bool VerifyConnected();

  // Add a setup routine that's called on intialization and when a reset is
  // detected.
  void OnInit(std::function<void()> callback);

  // Write a new output to the speed controller.
  void Write(Output output, units::millisecond_t timeout = kCANTimeout);

 private:
  const Named& parent_;

  ctre::TalonSRX& esc_;

  std::vector<std::function<void()>> on_inits_;

  TalonSRXConfigHelper* config_;
  GainsHelper* gains_;

  TalonSRXConfig config_cache_;
  Gains gains_cache_;
};

// Helper for TalonFX speed controllers.
//
// Handles motor setup, configurations, gains, outputting, and CAN status frame
// usage.
class TalonFXHelper {
 public:
  TalonFXHelper(const Named& parent, ctre::TalonFX& esc,
                TalonFXConfigHelper* config, GainsHelper* gains);

  ~TalonFXHelper();

  // Call setup routines and write initial configs and gains.
  void Setup(units::millisecond_t timeout = kCANTimeout);

  // Disable given status frames.
  void DisableStatusFrames(
      std::initializer_list<ctre::StatusFrameEnhanced> frames,
      units::millisecond_t timeout = kCANTimeout);

  // Verify that the speed controller connected.
  bool VerifyConnected();

  // Add a setup routine that's called on intialization and when a reset is
  // detected.
  void OnInit(std::function<void()> callback);

  // Write a new output to the speed controller.
  void Write(Output output, units::millisecond_t timeout = kCANTimeout);

 private:
  const Named& parent_;

  ctre::TalonFX& esc_;

  std::vector<std::function<void()>> on_inits_;

  TalonFXConfigHelper* config_;
  GainsHelper* gains_;

  TalonFXConfig config_cache_;
  Gains gains_cache_;
};

// Helper for SparkMAX speed controllers.
//
// Handles motor setup, configurations, gains, outputting, and CAN status frame
// usage.
class SparkMAXHelper {
 public:
  SparkMAXHelper(const Named& parent, rev::CANSparkMax& esc,
                 SparkMAXConfigHelper* config, GainsHelper* gains);

  ~SparkMAXHelper();

  // Get the Spark MAX pid controller.
  rev::SparkMaxPIDController& pid_controller() { return pid_controller_; }

  // Get the Spark MAX integrated encoder.
  rev::SparkMaxRelativeEncoder& encoder() { return encoder_; }

  // Call setup routines and write initial configs and gains.
  void Setup(units::millisecond_t timeout = kCANTimeout);

  // Disable given status frames.
  void DisableStatusFrames(
      std::initializer_list<rev::CANSparkMaxLowLevel::PeriodicFrame> frames);

  // Verify that the speed controller connected.
  bool VerifyConnected();

  // Add a setup routine that's called on intialization and when a reset is
  // detected.
  void OnInit(std::function<void()> callback);

  // Write a new output to the speed controller.
  void Write(Output output);

 private:
  const Named& parent_;

  rev::CANSparkMax& esc_;
  rev::SparkMaxPIDController pid_controller_;
  rev::SparkMaxRelativeEncoder encoder_;

  std::vector<std::function<void()>> on_inits_;

  SparkMAXConfigHelper* config_;
  GainsHelper* gains_;

  SparkMAXConfig config_cache_;
  Gains gains_cache_;
};

}  // namespace frc846::motor

#endif  // FRC846_MOTOR_HELPER_H_