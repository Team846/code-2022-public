#include "frc846/motor/helper.h"

namespace frc846::motor {

constexpr ctre::ControlMode CTREControlMode(ControlMode mode) {
  switch (mode) {
    case ControlMode::Percent:
      return ctre::ControlMode::PercentOutput;
    case ControlMode::Velocity:
      return ctre::ControlMode::Velocity;
    case ControlMode::Position:
      return ctre::ControlMode::Position;
    case ControlMode::Current:
      return ctre::ControlMode::Current;
    default:
      throw std::runtime_error("unsupported control type");
  }
}

constexpr rev::CANSparkMax::ControlType RevControlMode(ControlMode mode) {
  switch (mode) {
    case ControlMode::Percent:
      return rev::CANSparkMax::ControlType::kDutyCycle;
    case ControlMode::Velocity:
      return rev::CANSparkMax::ControlType::kVelocity;
    case ControlMode::Position:
      return rev::CANSparkMax::ControlType::kPosition;
    case ControlMode::Current:
      return rev::CANSparkMax::ControlType::kCurrent;
    default:
      throw std::runtime_error("unsupported control type");
  }
}

void CheckOk(const Named& named, ctre::ErrorCode err, std::string_view field) {
  if (err != ctre::ErrorCode::OK) {
    named.Error("Unable to update {}", field);
  }
}

void CheckOk(const Named& named, rev::REVLibError err, std::string_view field) {
  if (err != rev::REVLibError::kOk) {
    named.Error("Unable to update {}", field);
  }
}

void CTREDisableStatusFrames(
    const Named& helper, ctre::BaseMotorController& esc,
    std::initializer_list<ctre::StatusFrameEnhanced> frames,
    units::millisecond_t timeout) {
  for (auto f : frames) {
    // Max frame period is 255 ms
    // https://docs.ctre-phoenix.com/en/stable/ch18_CommonAPI.html
    auto err = esc.SetStatusFramePeriod(f, 255, timeout.to<double>());
    CheckOk(helper, err, "status_frame");
  }
}

void CTRESetup(const Named& helper, ctre::BaseMotorController& esc,
               units::millisecond_t timeout) {
  esc.ConfigFactoryDefault(timeout.to<double>());
  esc.SetNeutralMode(ctre::NeutralMode::Coast);

  // Disable unused status frames
  CTREDisableStatusFrames(helper, esc,
                          {
                              ctre::StatusFrameEnhanced::Status_3_Quadrature,
                              ctre::StatusFrameEnhanced::Status_4_AinTempVbat,
                              ctre::StatusFrameEnhanced::Status_8_PulseWidth,
                              ctre::StatusFrameEnhanced::Status_10_MotionMagic,
                              ctre::StatusFrameEnhanced::Status_12_Feedback1,
                              ctre::StatusFrameEnhanced::Status_13_Base_PIDF0,
                              ctre::StatusFrameEnhanced::Status_14_Turn_PIDF1,
                          },
                          timeout);
}

bool CTREVerifyConnected(ctre::BaseMotorController& esc) {
  // CTRE devices return -1 firmware version when not connected
  return esc.GetFirmwareVersion() != -1;
}

VictorSPXHelper::VictorSPXHelper(const Named& parent, ctre::VictorSPX& esc,
                                 VictorSPXConfigHelper* config)
    : parent_(parent), esc_(esc), config_(config) {}

VictorSPXHelper::~VictorSPXHelper() {
  if (config_ != nullptr) {
    delete config_;
    config_ = nullptr;
  }
}

void VictorSPXHelper::Setup(units::millisecond_t timeout) {
  CTRESetup(parent_, esc_, timeout);

  if (config_ != nullptr) {
    config_->Write(esc_, config_cache_, timeout, true);
  }

  for (auto c : on_inits_) {
    c();
  }
}

void VictorSPXHelper::DisableStatusFrames(
    std::initializer_list<ctre::StatusFrameEnhanced> frames,
    units::millisecond_t timeout) {
  CTREDisableStatusFrames(parent_, esc_, frames, timeout);
}

bool VictorSPXHelper::VerifyConnected() { return CTREVerifyConnected(esc_); }

void VictorSPXHelper::OnInit(std::function<void()> callback) {
  on_inits_.push_back(callback);
}

void VictorSPXHelper::Write(Output output, units::millisecond_t timeout) {
  if (esc_.HasResetOccurred()) {
    parent_.Warn("Reset detected!! Rewriting config...");
    Setup(kCANTimeout);
  }

  if (config_ != nullptr) {
    config_->Write(esc_, config_cache_, timeout);
    esc_.Set(CTREControlMode(output.mode), output.value);
  }
}

TalonSRXHelper::TalonSRXHelper(const Named& parent, ctre::TalonSRX& esc,
                               TalonSRXConfigHelper* config, GainsHelper* gains)
    : parent_(parent), esc_(esc), config_(config), gains_(gains) {}

TalonSRXHelper::~TalonSRXHelper() {
  if (config_ != nullptr) {
    delete config_;
    config_ = nullptr;
  }
  if (gains_ != nullptr) {
    delete gains_;
    gains_ = nullptr;
  }
}

void TalonSRXHelper::Setup(units::millisecond_t timeout) {
  CTRESetup(parent_, esc_, timeout);

  if (config_ != nullptr) {
    config_->Write(esc_, config_cache_, timeout, true);
  }
  if (gains_ != nullptr) {
    gains_->Write(esc_, gains_cache_, timeout, true);
  }

  for (auto c : on_inits_) {
    c();
  }
}

void TalonSRXHelper::DisableStatusFrames(
    std::initializer_list<ctre::StatusFrameEnhanced> frames,
    units::millisecond_t timeout) {
  CTREDisableStatusFrames(parent_, esc_, frames, timeout);
}

bool TalonSRXHelper::VerifyConnected() { return CTREVerifyConnected(esc_); }

void TalonSRXHelper::OnInit(std::function<void()> callback) {
  on_inits_.push_back(callback);
}

void TalonSRXHelper::Write(Output output, units::millisecond_t timeout) {
  if (esc_.HasResetOccurred()) {
    parent_.Warn("Reset detected!! Rewriting config...");
    Setup(kCANTimeout);
  }

  if (config_ != nullptr) {
    config_->Write(esc_, config_cache_, timeout);
  }
  if (gains_ != nullptr) {
    gains_->Write(esc_, gains_cache_, timeout);
  }
  esc_.Set(CTREControlMode(output.mode), output.value);
}

TalonFXHelper::TalonFXHelper(const Named& parent, ctre::TalonFX& esc,
                             TalonFXConfigHelper* config, GainsHelper* gains)
    : parent_(parent), esc_(esc), config_(config), gains_(gains) {}

TalonFXHelper::~TalonFXHelper() {
  if (config_ != nullptr) {
    delete config_;
    config_ = nullptr;
  }
  if (gains_ != nullptr) {
    delete gains_;
    gains_ = nullptr;
  }
}

void TalonFXHelper::Setup(units::millisecond_t timeout) {
  CTRESetup(parent_, esc_, timeout);

  if (config_ != nullptr) {
    config_->Write(esc_, config_cache_, timeout, true);
  }
  if (gains_ != nullptr) {
    gains_->Write(esc_, gains_cache_, timeout, true);
  }

  for (auto c : on_inits_) {
    c();
  }
}

void TalonFXHelper::DisableStatusFrames(
    std::initializer_list<ctre::StatusFrameEnhanced> frames,
    units::millisecond_t timeout) {
  CTREDisableStatusFrames(parent_, esc_, frames, timeout);
}

bool TalonFXHelper::VerifyConnected() { return CTREVerifyConnected(esc_); }

void TalonFXHelper::OnInit(std::function<void()> callback) {
  on_inits_.push_back(callback);
}

void TalonFXHelper::Write(Output output, units::millisecond_t timeout) {
  if (esc_.HasResetOccurred()) {
    parent_.Warn("Reset detected!! Rewriting config...");
    Setup(kCANTimeout);
  }

  if (config_ != nullptr) {
    config_->Write(esc_, config_cache_, timeout);
  }
  if (gains_ != nullptr) {
    gains_->Write(esc_, gains_cache_, timeout);
  }
  esc_.Set(CTREControlMode(output.mode), output.value);
}

SparkMAXHelper::SparkMAXHelper(const Named& parent, rev::CANSparkMax& esc,
                               SparkMAXConfigHelper* config, GainsHelper* gains)
    : parent_(parent),
      esc_(esc),
      pid_controller_(esc.GetPIDController()),
      encoder_(esc.GetEncoder()),
      config_(config),
      gains_(gains) {}

SparkMAXHelper::~SparkMAXHelper() {
  if (config_ != nullptr) {
    delete config_;
    config_ = nullptr;
  }
  if (gains_ != nullptr) {
    delete gains_;
    gains_ = nullptr;
  }
}

void SparkMAXHelper::Setup(units::millisecond_t timeout) {
  esc_.RestoreFactoryDefaults();
  esc_.SetCANTimeout(timeout.to<double>());
  esc_.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

  if (config_ != nullptr) {
    config_->Write(esc_, pid_controller_, config_cache_, true);
  }
  if (gains_ != nullptr) {
    gains_->Write(pid_controller_, gains_cache_, true);
  }

  for (auto c : on_inits_) {
    c();
  }

  esc_.BurnFlash();
}

void SparkMAXHelper::DisableStatusFrames(
    std::initializer_list<rev::CANSparkMaxLowLevel::PeriodicFrame> frames) {
  for (auto f : frames) {
    // Max frame period is 65535ms
    // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
    auto err = esc_.SetPeriodicFramePeriod(f, 65535);
    CheckOk(parent_, err, "status frame");
  }
}

void SparkMAXHelper::OnInit(std::function<void()> callback) {
  on_inits_.push_back(callback);
}

bool SparkMAXHelper::VerifyConnected() {
  // Spark MAX returns 0 firmware version when not connected

  // TODO this is stupid...
  // GetFirmwareVersion sometimes returns 0 the first time you call it even when
  // connected, so call it twice!
  esc_.GetFirmwareVersion();

  return esc_.GetFirmwareVersion() != 0;
}

void SparkMAXHelper::Write(Output output) {
  if (esc_.GetStickyFault(rev::CANSparkMax::FaultID::kHasReset)) {
    parent_.Warn("Reset detected!! Rewriting config...");
    Setup(kCANTimeout);
    esc_.ClearFaults();
  }

  if (config_ != nullptr) {
    config_->Write(esc_, pid_controller_, config_cache_);
  }
  if (gains_ != nullptr) {
    gains_->Write(pid_controller_, gains_cache_);
  }

  auto value = output.value;

  // Clamp output value to peak output range if using open loop.
  //
  // Configuring the output range of the PIDController in SparkMAXConfigHelper
  // only affects closed loop control.
  if (output.mode == ControlMode::Percent) {
    auto peak_output = config_->peak_output_.value();

    value = std::max(value, -peak_output);
    value = std::min(value, +peak_output);
  }

  pid_controller_.SetReference(value, RevControlMode(output.mode));
}

}  // namespace frc846::motor