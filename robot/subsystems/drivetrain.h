#ifndef ROBOT_SUBSYSTEMS_DRIVETRAIN_H_
#define ROBOT_SUBSYSTEMS_DRIVETRAIN_H_

#include <AHRS.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <array>
#include <variant>

#include "frc846/grapher.h"
#include "frc846/math.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "frc846/swerve_odometry.h"
#include "frc846/wpilib/time.h"
#include "robot/ports.h"
#include "robot/subsystems/limelight.h"
#include "robot/subsystems/swerve_module.h"

struct DrivetrainReadings {
  frc846::Position pose;
  units::degrees_per_second_t angular_velocity;
  frc846::Vector2D<units::feet_per_second_t> velocity;

  units::degree_t limelight_tx;  // intake limelight tx
  bool limelight_ball_exists;    // intake limelight tv
};

// Robot vs field oriented translation control.
enum DrivetrainTranslationReference { kRobot, kField };

// Position control of drivetrain steering.
using DrivetrainRotationPosition = units::degree_t;

// Velocity control of drivetrain steering.
using DrivetrainRotationVelocity = units::degrees_per_second_t;

// Drivetrain rotation target.
using DrivetrainRotation =
    std::variant<DrivetrainRotationPosition, DrivetrainRotationVelocity>;

struct DrivetrainTarget {
  units::feet_per_second_t v_x;
  units::feet_per_second_t v_y;
  DrivetrainTranslationReference translation_reference;

  DrivetrainRotation rotation;
};

class DrivetrainSubsystem
    : public frc846::Subsystem<DrivetrainReadings, DrivetrainTarget> {
 public:
  DrivetrainSubsystem();

  // Number of swerve modules (avoid hardcoding 4 in loops and such).
  static constexpr int kModuleCount = 4;

  // Max drivetrain speed (Falcon500 SDS Mk4i L1 -> 13.5 theoretical).
  frc846::Pref<units::feet_per_second_t> max_speed_{*this, "max_speed", 13_fps};

  // How much to scale down speed when in slow mode.
  frc846::Pref<double> slow_mode_percent_{*this, "slow_mode_percent", 0.5};

  // Max turning speed.
  units::degrees_per_second_t max_omega() const {
    return max_speed_.value() / module_radius_ * 1_rad *
           percent_max_omega_.value();
  }

  // Shoot while driving fudge factor
  frc846::Pref<double> moving_fudge{*this, "moving_fudge", 1.0};

  // Max drivetrain acceleration for trajectory generation.
  frc846::Pref<units::feet_per_second_squared_t> max_acceleration_{
      *this, "max_acceleration", 10_fps_sq};

  // Max drivetrain deceleration for trajectory generation.
  frc846::Pref<units::feet_per_second_squared_t> max_deceleration_{
      *this, "max_deceleration", 10_fps_sq};

  // Lookahead distance during trajectory following.
  frc846::Pref<units::inch_t> extrapolation_distance_{
      *this, "extrapolation_distance", 8_in};

  // Max angular veloity and velocity tresholds to be considered "not moving"
  // when shooting.
  frc846::Pref<units::degrees_per_second_t> angular_velocity_threshold_{
      *this, "angular_velocity_threshold", 6_deg_per_s};
  frc846::Pref<units::feet_per_second_t> velocity_threshold_{
      *this, "velocity_threshold", 0.5_fps};

  // Zero the modules with their CANCoders.
  void ZeroModules();

  // Zero bearing with the gyro.
  void ZeroBearing();

  // Zero bearing and reset odometry to zero.
  void ZeroOdometry();

  // Set odometry point.
  void SetPoint(frc846::Vector2D<units::foot_t> point);

  // Set bearing.
  void SetBearing(units::degree_t bearing);

  // Set odometry position with limelight target.
  void LocalizeOdometry(units::foot_t hub_distance, units::degree_t tx);

  // Convert a translation vector and the drivetrain angular velocity to the
  // individual module outputs.
  static std::array<frc846::Vector2D<units::feet_per_second_t>, kModuleCount>
  SwerveControl(frc846::Vector2D<units::feet_per_second_t> translation,
                units::degrees_per_second_t rotation_speed, units::inch_t width,
                units::inch_t height, units::inch_t radius,
                units::feet_per_second_t max_speed);

  DrivetrainTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  // Drivetrain dimensions.
  frc846::Pref<units::inch_t> width_{*this, "width", 21.75_in};
  frc846::Pref<units::inch_t> height_{*this, "height", 21.75_in};

  // How much to scale the max turning speed by.
  frc846::Pref<double> percent_max_omega_{*this, "percent_max_omega", 0.45};

  // Distance from center of robot to module.
  units::inch_t module_radius_ =
      units::math::sqrt(units::math::pow<2>(width_.value() / 2) +
                        units::math::pow<2>(height_.value() / 2));

  // Wheel radius for odometry. 4" wheels.
  frc846::Pref<units::inch_t> wheel_radius_{*this, "wheel_radius", 1.926_in};

  // Rotation position gains.
  frc846::Named bearing_gains_named_{*this, "bearing_gains"};
  frc846::Pref<double> bearing_gains_p_{bearing_gains_named_, "p", 7};
  frc846::Pref<double> bearing_gains_d_{bearing_gains_named_, "d", -4};

  // Pose graphers.
  frc846::Named pose_named_{*this, "pose"};
  frc846::Grapher<units::foot_t> pose_x_graph_{pose_named_, "x"};
  frc846::Grapher<units::foot_t> pose_y_graph_{pose_named_, "y"};
  frc846::Grapher<units::degree_t> pose_bearing_graph{pose_named_, "bearing"};

  // Velocity graphers.
  frc846::Named velocity_named_{*this, "velocity"};
  frc846::Grapher<units::feet_per_second_t> v_x_graph_{velocity_named_, "v_x"};
  frc846::Grapher<units::feet_per_second_t> v_y_graph_{velocity_named_, "v_y"};

  // Limelight graphers.
  frc846::Named limelight_named_{*this, "limelight"};
  frc846::Grapher<bool> limelight_ball_exists_graph_{limelight_named_,
                                                     "ball_exists"};
  frc846::Grapher<units::degree_t> limelight_tx_graph_{limelight_named_, "tx"};

  // Target graphers.
  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<units::feet_per_second_t> target_v_x_graph_{target_named_,
                                                              "v_x"};
  frc846::Grapher<units::feet_per_second_t> target_v_y_graph_{target_named_,
                                                              "v_y"};
  frc846::Grapher<std::string> target_translation_reference_graph_{
      target_named_,
      "translation_reference",
  };
  frc846::Grapher<units::degree_t> target_rotation_position_graph_{
      target_named_, "rotation_position"};
  frc846::Grapher<units::degrees_per_second_t> target_rotation_velocity_graph_{
      target_named_, "rotation_velocity"};

  frc846::SwerveOdometry odometry_;

  // How much to offset the bearing by.
  units::angle::degree_t bearing_offset_;

  frc846::Named drive_esc_named_{*this, "drive_esc"};
  frc846::Named steer_esc_named_{*this, "steer_esc"};

  frc846::motor::TalonFXConfigHelper* drive_esc_config_helper_ =
      new frc846::motor::TalonFXConfigHelper{
          drive_esc_named_,
          {
              1.0,  /* peak_output */
              12_V, /* voltage_comp_saturation */

              80_A, /* supply_peak_current_limit */
              1_s,  /* supply_peak_current_duration */
              40_A, /* supply_continuous_current_limit */

              0_A,   /* stator_peak_current_limit */
              0_s,   /* stator_peak_current_duration */
              250_A, /* stator_continuous_current_limit */
          },

      };
  frc846::motor::GainsHelper* drive_esc_gains_helper_ =
      new frc846::motor::GainsHelper{
          drive_esc_named_,
          {
              0.0083, /* p */
              0,      /* i */
              0,      /* d */
              0.053,  /* f */
              0,      /* max_integral_accumulator */
          },
      };
  frc846::motor::TalonFXConfigHelper* steer_esc_config_helper_ =
      new frc846::motor::TalonFXConfigHelper{
          steer_esc_named_,
          {
              1.0,  /* peak_output */
              12_V, /* voltage_comp_saturation */

              40_A, /* supply_peak_current_limit */
              1_s,  /* supply_peak_current_duration */
              30_A, /* supply_continuous_current_limit */

              40_A, /* stator_peak_current_limit */
              1_s,  /* stator_peak_current_duration */
              30_A, /* stator_continuous_current_limit */
          },

      };
  frc846::motor::GainsHelper* steer_esc_gains_helper_ =
      new frc846::motor::GainsHelper{
          steer_esc_named_,
          {
              0.12, /* p */
              0,    /* i */
              0,    /* d */
              0,    /* f */
              0,    /* max_integral_accumulator */
          },
      };

  frc846::motor::Converter<units::foot_t> drive_converter_{
      frc846::motor::kTalonPeriod,
      frc846::motor::kTalonFXSensorTicks,
      (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0) *  // L1 gearing
          frc846::Circumference(wheel_radius_.value()),
  };
  frc846::motor::Converter<units::degree_t> steer_converter_{
      frc846::motor::kTalonPeriod,
      frc846::motor::kTalonFXSensorTicks,
      (7.0 / 150.0) * 1_tr,
  };

  SwerveModuleSubsystem module_fl_{
      *this,
      "FL",
      243.95_deg,
      drive_esc_config_helper_,
      drive_esc_gains_helper_,
      steer_esc_config_helper_,
      steer_esc_gains_helper_,
      drive_converter_,
      steer_converter_,
      ports::drivetrain::kFLDrive_CANID,
      ports::drivetrain::kFLSteer_CANID,
      ports::drivetrain::kFLCANCoder_CANID,
  };

  SwerveModuleSubsystem module_fr_{
      *this,
      "FR",
      281.42_deg,
      drive_esc_config_helper_,
      drive_esc_gains_helper_,
      steer_esc_config_helper_,
      steer_esc_gains_helper_,
      drive_converter_,
      steer_converter_,
      ports::drivetrain::kFRDrive_CANID,
      ports::drivetrain::kFRSteer_CANID,
      ports::drivetrain::kFRCANCoder_CANID,
  };

  SwerveModuleSubsystem module_bl_{
      *this,
      "BL",
      92.48_deg,
      drive_esc_config_helper_,
      drive_esc_gains_helper_,
      steer_esc_config_helper_,
      steer_esc_gains_helper_,
      drive_converter_,
      steer_converter_,
      ports::drivetrain::kBLDrive_CANID,
      ports::drivetrain::kBLSteer_CANID,
      ports::drivetrain::kBLCANCoder_CANID,
  };

  SwerveModuleSubsystem module_br_{
      *this,
      "BR",
      69.34_deg,
      drive_esc_config_helper_,
      drive_esc_gains_helper_,
      steer_esc_config_helper_,
      steer_esc_gains_helper_,
      drive_converter_,
      steer_converter_,
      ports::drivetrain::kBRDrive_CANID,
      ports::drivetrain::kBRSteer_CANID,
      ports::drivetrain::kBRCANCoder_CANID,
  };

  SwerveModuleSubsystem* modules_all_[kModuleCount]{&module_fl_, &module_fr_,
                                                    &module_bl_, &module_br_};

  AHRS gyro_{frc::SPI::Port::kMXP};

  std::shared_ptr<nt::NetworkTable> limelight_table_ =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight_intake");

  DrivetrainReadings GetNewReadings() override;

  void WriteToHardware(DrivetrainTarget target) override;
};

#endif  // ROBOT_SUBSYSTEMS_DRIVETRAIN_H_