#ifndef ROBOT_SUBSYSTEMS_CLIMBER_H_
#define ROBOT_SUBSYSTEMS_CLIMBER_H_

#include <frc/Solenoid.h>
#include <rev/CANSparkMax.h>

#include "frc846/grapher.h"
#include "frc846/motor/converter.h"
#include "frc846/motor/helper.h"
#include "frc846/subsystem.h"
#include "frc846/wpilib/time.h"
#include "robot/ports.h"
#include "robot/subsystems/climber_arm.h"

struct ClimberReadings {
  ClimberArmReadings left;
  ClimberArmReadings right;
};

struct ClimberTarget {
  // [-1, 1] where + is climbing and - is swinging
  double left_speed;
  double right_speed;
};

struct ClimberSetpointPref : public frc846::Named {
  ClimberSetpointPref(const frc846::Named& parent, std::string_view name,
                      units::degree_t fallback_position, double fallback_speed)
      : frc846::Named{parent, name},
        position{*this, "position", fallback_position},
        speed{*this, "speed", fallback_speed} {}

  frc846::Pref<units::degree_t> position;
  frc846::Pref<double> speed;
};

class ClimberSubsystem
    : public frc846::Subsystem<ClimberReadings, ClimberTarget> {
 public:
  ClimberSubsystem();

  // Manual control speeds
  frc846::Pref<double> manual_climb_speed_{*this, "manual_climb_speed", 0.25};
  frc846::Pref<double> manual_swing_speed_{*this, "manual_swing_speed", -0.3};

  // Bang-bang position control tolerance
  frc846::Pref<units::degree_t> position_control_tolerance_{
      *this, "position_control_tolerance", 5_deg};

  frc846::Named setpoints_named_{*this, "setpoints"};

  // Setpoint to stow the climber before the match.
  ClimberSetpointPref setpoints_pre_stow_{setpoints_named_, "pre_stow", 0_deg,
                                          0.2};

  // Setpoint to stow the climber during the match, out of the intake's way.
  ClimberSetpointPref setpoints_match_stow_{setpoints_named_, "match_stow",
                                            -20_deg, 0.3};

  // Setpoint to deploy hooks and put arms up to align to mid bar.
  ClimberSetpointPref setpoints_align_{setpoints_named_, "align", -110_deg,
                                       0.4};

  // Setpoint to climb from mid to high.
  ClimberSetpointPref setpoints_climb_high_{setpoints_named_, "climb_high",
                                            65_deg, 0.35};

  // Setpoint to release from mid bar.
  ClimberSetpointPref setpoints_swing_high_{setpoints_named_, "swing_high",
                                            65_deg, 0.0};

  // Setpoint to climb from high to traversal.
  ClimberSetpointPref setpoints_climb_traversal_{
      setpoints_named_, "climb_traversal", 230_deg, 0.7};

  // Setpoint to release from high bar.
  ClimberSetpointPref setpoints_swing_traversal_{
      setpoints_named_, "swing_traversal", 240_deg, 0.0};

  // Zero both climber arms.
  void Zero();

  ClimberTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::Named esc_named_{*this, "esc"};

  frc846::motor::SparkMAXConfigHelper* esc_config_helper_ =
      new frc846::motor::SparkMAXConfigHelper{
          esc_named_,
          {
              1.0,  /* peak_output */
              10_V, /* voltage_comp_saturation */

              60_A, /* current_limit */
          },
      };

  ClimberArmSubsystem left_arm_{
      *this,
      "left",
      true, /* inverted */
      esc_config_helper_,
      ports::climber::kLeftLeader_CANID,
      ports::climber::kLeftFollower_CANID,
  };

  ClimberArmSubsystem right_arm_{
      *this,
      "right",
      false, /* inverted */
      esc_config_helper_,
      ports::climber::kRightLeader_CANID,
      ports::climber::kRightFollower_CANID,
  };

  ClimberReadings GetNewReadings() override;

  void WriteToHardware(ClimberTarget target) override;
};

using OptionalClimberSubsystem =
    frc846::OptionalSubsystem<ClimberSubsystem, ClimberReadings, ClimberTarget>;

#endif  // ROBOT_SUBSYSTEMS_CLIMBER_H_