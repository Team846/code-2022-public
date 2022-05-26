#include "frc846/motor/config.h"

#include <string_view>

#include "frc846/motor/helper.h"

namespace frc846::motor {

VictorSPXConfigHelper::VictorSPXConfigHelper(const Named& parent,
                                             VictorSPXConfig config)
    : Named{parent, "config"},
      peak_output_{*this, "peak_output", config.peak_output},
      voltage_comp_saturation_{*this, "voltage_comp_saturation",
                               config.voltage_comp_saturation} {}

void VictorSPXConfigHelper::Write(ctre::VictorSPX& esc, VictorSPXConfig& cache,
                                  units::millisecond_t timeout,
                                  bool ignore_cache) {
  double timeout_ms = timeout.to<double>();

  if (ignore_cache || cache.peak_output != peak_output_.value()) {
    auto err = esc.ConfigPeakOutputForward(peak_output_.value(), timeout_ms);
    CheckOk(*this, err, "peak_output_forward");
    err = esc.ConfigPeakOutputReverse(-peak_output_.value(), timeout_ms);
    CheckOk(*this, err, "peak_output_reverse");

    cache.peak_output = peak_output_.value();
  }
  if (ignore_cache ||
      cache.voltage_comp_saturation != voltage_comp_saturation_.value()) {
    auto err = esc.ConfigVoltageCompSaturation(
        voltage_comp_saturation_.value().to<double>(), timeout_ms);
    CheckOk(*this, err, "voltage_comp_saturation");

    esc.EnableVoltageCompensation(true);

    cache.voltage_comp_saturation = voltage_comp_saturation_.value();
  }
}

TalonSRXConfigHelper::TalonSRXConfigHelper(const Named& parent,
                                           TalonSRXConfig config)
    : Named{parent, "config"},
      peak_output_{*this, "peak_output", config.peak_output},
      voltage_comp_saturation_{*this, "voltage_comp_saturation",
                               config.voltage_comp_saturation},

      peak_current_limit_{*this, "peak_current_limit",
                          config.peak_current_limit},
      peak_current_duration_{*this, "peak_current_duration",
                             config.peak_current_duration},
      continuous_current_limit_{*this, "continuous_current_limit",
                                config.continuous_current_limit} {}

void TalonSRXConfigHelper::Write(ctre::TalonSRX& esc, TalonSRXConfig& cache,
                                 units::millisecond_t timeout,
                                 bool ignore_cache) {
  double timeout_ms = timeout.to<double>();

  if (ignore_cache || cache.peak_output != peak_output_.value()) {
    auto err = esc.ConfigPeakOutputForward(peak_output_.value(), timeout_ms);
    CheckOk(*this, err, "peak_output_forward");
    err = esc.ConfigPeakOutputReverse(-peak_output_.value(), timeout_ms);
    CheckOk(*this, err, "peak_output_reverse");

    cache.peak_output = peak_output_.value();
  }
  if (ignore_cache ||
      cache.voltage_comp_saturation != voltage_comp_saturation_.value()) {
    auto err = esc.ConfigVoltageCompSaturation(
        voltage_comp_saturation_.value().to<double>(), timeout_ms);
    CheckOk(*this, err, "voltage_comp_saturation");

    esc.EnableVoltageCompensation(true);

    cache.voltage_comp_saturation = voltage_comp_saturation_.value();
  }
  if (ignore_cache || cache.peak_current_limit != peak_current_limit_.value()) {
    auto err = esc.ConfigPeakCurrentLimit(
        peak_current_limit_.value().to<double>(), timeout_ms);
    CheckOk(*this, err, "peak_current_limit");

    cache.peak_current_limit = peak_current_limit_.value();
  }
  if (ignore_cache ||
      cache.peak_current_duration != peak_current_duration_.value()) {
    auto err = esc.ConfigPeakCurrentDuration(
        peak_current_duration_.value().to<double>(), timeout_ms);
    CheckOk(*this, err, "peak_current_duration");

    cache.peak_current_duration = peak_current_duration_.value();
  }
  if (ignore_cache ||
      cache.continuous_current_limit != continuous_current_limit_.value()) {
    auto err = esc.ConfigContinuousCurrentLimit(
        continuous_current_limit_.value().to<double>(), timeout_ms);
    CheckOk(*this, err, "continuous_current_limit");

    cache.continuous_current_limit = continuous_current_limit_.value();
  }
}

TalonFXConfigHelper::TalonFXConfigHelper(const Named& parent,
                                         TalonFXConfig config)
    : Named{parent, "config"},
      peak_output_{*this, "peak_output", config.peak_output},
      voltage_comp_saturation_{*this, "voltage_comp_saturation",
                               config.voltage_comp_saturation},

      supply_peak_current_limit_{*this, "supply_peak_current_limit",
                                 config.supply_peak_current_limit},
      supply_peak_current_duration_{*this, "supply_peak_current_duration",
                                    config.supply_peak_current_duration},
      supply_continuous_current_limit_{*this, "supply_continuous_current_limit",
                                       config.supply_continuous_current_limit},

      stator_peak_current_limit_{*this, "stator_peak_current_limit",
                                 config.stator_peak_current_limit},
      stator_peak_current_duration_{*this, "stator_peak_current_duration",
                                    config.stator_peak_current_duration},
      stator_continuous_current_limit_{*this, "stator_continuous_current_limit",
                                       config.stator_continuous_current_limit} {
}

void TalonFXConfigHelper::Write(ctre::TalonFX& esc, TalonFXConfig& cache,
                                units::millisecond_t timeout,
                                bool ignore_cache) {
  double timeout_ms = timeout.to<double>();

  if (ignore_cache || cache.peak_output != peak_output_.value()) {
    auto err = esc.ConfigPeakOutputForward(peak_output_.value(), timeout_ms);
    CheckOk(*this, err, "peak_output_forward");
    err = esc.ConfigPeakOutputReverse(-peak_output_.value(), timeout_ms);
    CheckOk(*this, err, "peak_output_reverse");

    cache.peak_output = peak_output_.value();
  }
  if (ignore_cache ||
      cache.voltage_comp_saturation != voltage_comp_saturation_.value()) {
    auto err = esc.ConfigVoltageCompSaturation(
        voltage_comp_saturation_.value().to<double>(), timeout_ms);
    CheckOk(*this, err, "voltage_comp_saturation");

    esc.EnableVoltageCompensation(true);

    cache.voltage_comp_saturation = voltage_comp_saturation_.value();
  }
  if (ignore_cache ||
      cache.supply_continuous_current_limit !=
          supply_continuous_current_limit_.value() ||
      cache.supply_peak_current_limit != supply_peak_current_limit_.value() ||
      cache.supply_peak_current_duration !=
          supply_peak_current_duration_.value()) {
    auto err = esc.ConfigSupplyCurrentLimit(
        {
            true, /* enable */
            supply_continuous_current_limit_.value()
                .to<double>(), /* currentLimit */
            supply_peak_current_limit_.value()
                .to<double>(), /* triggerThresholdCurrent */
            supply_peak_current_duration_.value()
                .to<double>(), /* triggerThresholdTime */
        },
        timeout_ms);

    CheckOk(*this, err, "supply_current_limit");

    cache.supply_continuous_current_limit =
        supply_continuous_current_limit_.value();
    cache.supply_peak_current_limit = supply_peak_current_limit_.value();
    cache.supply_peak_current_duration = supply_peak_current_duration_.value();
  }
  if (ignore_cache ||
      cache.stator_continuous_current_limit !=
          stator_continuous_current_limit_.value() ||
      cache.stator_peak_current_limit != stator_peak_current_limit_.value() ||
      cache.stator_peak_current_duration !=
          stator_peak_current_duration_.value()) {
    auto err = esc.ConfigStatorCurrentLimit(
        {
            true, /* enable */
            stator_continuous_current_limit_.value()
                .to<double>(), /* currentLimit */
            stator_peak_current_limit_.value()
                .to<double>(), /* triggerThresholdCurrent */
            stator_peak_current_duration_.value()
                .to<double>(), /* triggerThresholdTime */
        },
        timeout_ms);

    CheckOk(*this, err, "stator_current_limit");

    cache.stator_continuous_current_limit =
        stator_continuous_current_limit_.value();
    cache.stator_peak_current_limit = stator_peak_current_limit_.value();
    cache.stator_peak_current_duration = stator_peak_current_duration_.value();
  }
}

SparkMAXConfigHelper::SparkMAXConfigHelper(const Named& parent,
                                           SparkMAXConfig config)
    : Named{parent, "config"},
      peak_output_{*this, "peak_output", config.peak_output},
      voltage_comp_saturation_{*this, "voltage_comp_saturation",
                               config.voltage_comp_saturation},

      current_limit_{*this, "current_limit", config.current_limit} {}

void SparkMAXConfigHelper::Write(rev::CANSparkMax& esc,
                                 rev::SparkMaxPIDController& pid_controller,
                                 SparkMAXConfig& cache, bool ignore_cache) {
  if (ignore_cache || cache.peak_output != peak_output_.value()) {
    auto value = peak_output_.value();
    auto err = pid_controller.SetOutputRange(-value, value);
    CheckOk(*this, err, "output_range");

    cache.peak_output = peak_output_.value();
  }
  if (ignore_cache ||
      cache.voltage_comp_saturation != voltage_comp_saturation_.value()) {
    auto err = esc.EnableVoltageCompensation(
        voltage_comp_saturation_.value().to<double>());
    CheckOk(*this, err, "voltage_compensation");

    cache.voltage_comp_saturation = voltage_comp_saturation_.value();
  }
  if (ignore_cache || cache.current_limit != current_limit_.value()) {
    auto err = esc.SetSmartCurrentLimit(current_limit_.value().to<double>());
    CheckOk(*this, err, "smart_current_limit");

    cache.current_limit = current_limit_.value();
  }
}

}  // namespace frc846::motor
