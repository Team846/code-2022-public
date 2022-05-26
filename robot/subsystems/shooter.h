#ifndef ROBOT_SUBSYSTEMS_SHOOTER_H_
#define ROBOT_SUBSYSTEMS_SHOOTER_H_

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include "frc846/grapher.h"
#include "frc846/math.h"
#include "frc846/motor/converter.h"
#include "frc846/motor/helper.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "robot/ports.h"
#include "robot/subsystems/limelight.h"
#include "units/area.h"
#include "units/mass.h"
#include "units/math.h"

using SlugFootSq =
    units::unit_t<units::compound_unit<units::slug, units::square_foot>>;

struct ShooterReadings {
  units::revolutions_per_minute_t speed;  // average between left and right
  bool is_ready;                          // speed is within setpoint tolerance
};

struct ShooterTarget {
  units::revolutions_per_minute_t speed;
};

class ShooterSubsystem
    : public frc846::Subsystem<ShooterReadings, ShooterTarget> {
 public:
  ShooterSubsystem();

  // Enable shoot while driving.
  frc846::Pref<bool> should_compensate_rpm_{*this, "should_compensate_rpm_",
                                            false};

  // Shoot while driving fudge factor.
  frc846::Pref<double> shoot_fudge_{*this, "shoot_fudge", 1.0};

  frc846::Named presets_named_{*this, "presets"};

  // Preset value for low goal when against hub.
  frc846::Pref<units::revolutions_per_minute_t> preset_close_{
      presets_named_, "close", 1000_rpm};

  // Preset value for when on tarmac line.
  frc846::Pref<units::revolutions_per_minute_t> preset_mid_{presets_named_,
                                                            "mid", 3450_rpm};
  // Preset value for further than tarmac line.
  frc846::Pref<units::revolutions_per_minute_t> preset_far_{presets_named_,
                                                            "far", 5000_rpm};
  // Speed to eject wrong color ball (also idle speed).
  frc846::Pref<units::revolutions_per_minute_t> eject_speed_{presets_named_,
                                                             "eject", 800_rpm};

  frc846::Pref<units::revolutions_per_minute_t> load_speed_threshold_{
      presets_named_, "load_threshold", 900_rpm};

  frc846::Pref<units::degree_t> shooter_angle_{*this, "shooter_angle", 71_deg};

  frc846::Pref<units::second_t> max_tof_error_{*this, "max_tof_error", 0.25_s};

  // Estimate time the ball spends in the air.
  units::second_t GetTimeFlight(units::degree_t ty, units::foot_t distance);

  // Calculate the optimal speed to shoot at based on limelight ty and an
  // interpolation table.
  units::revolutions_per_minute_t GetOptimalRPM(
      units::degree_t target_vertical_angle);

  // Calculate ty from a distance
  units::degree_t DistanceToTy(units::foot_t distance,
                               units::degree_t mounting_angle,
                               units::foot_t mounting_height);

  // Calculate how much to offset the target shooter speed by when shooting
  // while driving.
  //
  // This method returns a pair of the offset speed as well as the theoreticall
  // hub position with drivetrain velocity factored in.
  std::pair<units::revolutions_per_minute_t, frc846::Vector2D<units::foot_t>>
  GetCompensatedRPM(
      frc846::Vector2D<units::feet_per_second_t> drivetrain_velocity,
      LimelightReadings limelight, units::degree_t mounting_angle,
      units::foot_t mounting_height);

  ShooterTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  units::revolutions_per_minute_t target_speed_;

  // Shooter speed interpolation quadratic coefficients
  // ty (deg) -> speed (rpm)
  frc846::Named interpolation_named_{*this, "interpolation"};
  frc846::Pref<double> a_{interpolation_named_, "a", 3.89637};
  frc846::Pref<double> b_{interpolation_named_, "b", -96.7526};
  frc846::Pref<double> c_{interpolation_named_, "c", 3550};

  // Shooting while moving parameters
  frc846::Pref<units::inch_t> ball_radius_{presets_named_, "ball radius",
                                           4.75_in};
  frc846::Pref<double> wheel_moment_of_inertia_{presets_named_, "MOI", 1.19};
  frc846::Pref<units::slug_t> ball_mass_{presets_named_, "ball_mass",
                                         0.0185_slug};
  frc846::Pref<units::inch_t> flywheel_radius_{presets_named_, "radius",
                                               1.5_in};

  frc846::Grapher<units::degree_t> latency_grapher_{*this, "latency"};

  // Fudge factor on left flywheel feedforward over right flywheel.
  frc846::Pref<double> left_bonus_factor_{*this, "left_bonus_factor", 1.015};

  // Speed error tolerance to shoot.
  frc846::Pref<units::revolutions_per_minute_t> tolerance_{*this, "tolerance",
                                                           100_rpm};

  frc846::Grapher<units::revolutions_per_minute_t> left_rpm_graph_{*this,
                                                                   "left_rpm"};
  frc846::Grapher<units::revolutions_per_minute_t> right_rpm_graph_{
      *this, "right_rpm"};
  frc846::Grapher<units::revolutions_per_minute_t> rpm_difference_graph_{
      *this, "rpm_difference"};

  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<units::revolutions_per_minute_t> target_speed_graph_{
      target_named_, "speed"};

  frc846::motor::Converter<units::turn_t> converter_{1_min, 1,
                                                     (12.0 / 24.0) * 1_tr};

  rev::CANSparkMax left_esc_{
      ports::shooter::kLeft_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };
  rev::CANSparkMax right_esc_{
      ports::shooter::kRight_CANID,
      rev::CANSparkMaxLowLevel::MotorType::kBrushless,
  };

  frc846::Named esc_named_{*this, "esc"};

  frc846::motor::SparkMAXConfigHelper* esc_config_helper_ =
      new frc846::motor::SparkMAXConfigHelper{
          esc_named_,
          {
              1.0,  /* peak_output */
              10_V, /* voltage_comp_saturation */

              40_A, /* current_limit */
          },
      };

  frc846::motor::GainsHelper* esc_gains_helper_ =
      new frc846::motor::GainsHelper{
          esc_named_,
          {
              0.0000025, /* p */
              0,         /* i */
              0,         /* d */
              0.0001063, /* f */
              0,         /* max_integral_accumulator */
          },
      };

  frc846::Named esc_left_named_{esc_named_, "left"};
  frc846::motor::SparkMAXHelper left_esc_helper_{
      esc_left_named_, left_esc_, esc_config_helper_, esc_gains_helper_};

  frc846::Named esc_right_named_{esc_named_, "right"};
  frc846::motor::SparkMAXHelper right_esc_helper_{
      esc_right_named_, right_esc_, esc_config_helper_, esc_gains_helper_};

  ShooterReadings GetNewReadings() override;

  void WriteToHardware(ShooterTarget target) override;
};

using OptionalShooterSubsystem =
    frc846::OptionalSubsystem<ShooterSubsystem, ShooterReadings, ShooterTarget>;

#endif  // ROBOT_SUBSYSTEMS_SHOOTER_H_