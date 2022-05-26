#include "frc846/motor/gains.h"

#include "frc846/motor/helper.h"

namespace frc846::motor {

GainsHelper::GainsHelper(Named& parent, Gains gains)
    : Named{parent, "gains"},
      p_{*this, "p", gains.p},
      i_{*this, "i", gains.i},
      d_{*this, "d", gains.d},
      f_{*this, "f", gains.f},
      max_integral_accumulator_{*this, "max_integral_accumulator",
                                gains.max_integral_accumulator} {}

void GainsHelper::Write(ctre::BaseTalon& esc, Gains& cache,
                        units::time::millisecond_t timeout, bool ignore_cache) {
  double timeout_ms = timeout.to<double>();

  if (ignore_cache || cache.p != p_.value()) {
    auto err = esc.Config_kP(kTalonSlotIdx, p_.value(), timeout_ms);
    CheckOk(*this, err, "p");

    cache.p = p_.value();
  }
  if (ignore_cache || cache.i != i_.value()) {
    auto err = esc.Config_kI(kTalonSlotIdx, i_.value(), timeout_ms);
    CheckOk(*this, err, "i");

    cache.i = i_.value();
  }
  if (ignore_cache || cache.d != d_.value()) {
    auto err = esc.Config_kD(kTalonSlotIdx, d_.value(), timeout_ms);
    CheckOk(*this, err, "d");

    cache.d = d_.value();
  }
  if (ignore_cache || cache.f != f_.value()) {
    auto err = esc.Config_kF(kTalonSlotIdx, f_.value(), timeout_ms);
    CheckOk(*this, err, "f");

    cache.f = f_.value();
  }
  if (ignore_cache ||
      cache.max_integral_accumulator != max_integral_accumulator_.value()) {
    auto err = esc.Config_IntegralZone(
        kTalonSlotIdx, max_integral_accumulator_.value(), timeout_ms);
    CheckOk(*this, err, "max integral accumulator");

    cache.max_integral_accumulator = max_integral_accumulator_.value();
  }
}

void GainsHelper::Write(rev::SparkMaxPIDController& pid_controller,
                        Gains& cache, bool ignore_cache) {
  if (ignore_cache || cache.p != p_.value()) {
    auto err = pid_controller.SetP(p_.value());
    CheckOk(*this, err, "p");

    cache.p = p_.value();
  }
  if (ignore_cache || cache.i != i_.value()) {
    auto err = pid_controller.SetI(i_.value());
    CheckOk(*this, err, "i");

    cache.i = i_.value();
  }
  if (ignore_cache || cache.d != d_.value()) {
    auto err = pid_controller.SetD(d_.value());
    CheckOk(*this, err, "d");

    cache.d = d_.value();
  }
  if (ignore_cache || cache.f != f_.value()) {
    auto err = pid_controller.SetFF(f_.value());
    CheckOk(*this, err, "f");

    cache.f = f_.value();
  }
  if (ignore_cache ||
      cache.max_integral_accumulator != max_integral_accumulator_.value()) {
    auto err = pid_controller.SetIMaxAccum(max_integral_accumulator_.value());
    CheckOk(*this, err, "max integral accumulator");

    cache.max_integral_accumulator = max_integral_accumulator_.value();
  }
}

}  // namespace frc846::motor