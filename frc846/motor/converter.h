#ifndef FRC846_MOTOR_CONVERTER_H_
#define FRC846_MOTOR_CONVERTER_H_

#include <units/angle.h>
#include <units/dimensionless.h>
#include <units/time.h>

#include <initializer_list>
#include <type_traits>

namespace frc846::motor {

using Tick = double;

// Talon time denominator in measurements.
//
// Plug into `native_per_time`
static units::second_t constexpr kTalonPeriod = 100_ms;

// TalonFX integrated encoder resolution (ticks/turn).
//
// Plug into `native_units_per_real_units`.
static Tick constexpr kTalonFXSensorTicks = 2048;

// Talon time denominator in measurements.
//
// Plug into `native_per_time`
static units::second_t constexpr kSparkMAXPeriod = 1_s;

// SparkMAX integrated encoder resolution (ticks/turn).
//
// Plug into `native_units_per_real_units`.
static Tick constexpr kSparkMAXSensorTicks = 1;

// Common conversions between real hardware values and native speed controller
// values.
template <typename X>
class Converter {
  using V = units::unit_t<units::compound_unit<typename X::unit_type,
                                               units::inverse<units::second>>>;

 public:
  Converter(units::second_t native_period, Tick native_sensor_ticks, X x)
      : native_period_(native_period),
        native_sensor_ticks_(native_sensor_ticks),
        x_(x) {}

  // Gets the real hardware position given the native measurement.RealUnit
  X NativeToRealPosition(Tick native) const {
    return native * x_ / native_sensor_ticks_;
  }

  // Gets the real hardware velocity given the native measurement.
  V NativeToRealVelocity(Tick native) const {
    return NativeToRealPosition(native) / native_period_;
  }

  // Gets the native measurement given the real hardware position.
  Tick RealToNativePosition(X real) const {
    return real * native_sensor_ticks_ / x_;
  }

  // Gets the native measurement given the real hardware velocity.
  Tick RealToNativeVelocity(V real) const {
    return RealToNativePosition(real * native_period_);
  }

 private:
  const units::second_t native_period_;
  const Tick native_sensor_ticks_;
  const X x_;
};

}  // namespace frc846::motor

#endif  // FRC846_MOTOR_CONVERTER_H_