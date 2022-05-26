#ifndef ROBOT_SUBSYSTEMS_FEEDER_H_
#define ROBOT_SUBSYSTEMS_FEEDER_H_

#include <frc/DriverStation.h>
#include <rev/CANSparkMax.h>
#include <rev/ColorSensorV3.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include "frc846/grapher.h"
#include "frc846/motor/converter.h"
#include "frc846/motor/helper.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "robot/ports.h"

// State of the ball in the feeder tower.
enum class FeederBallState {
  kNone,
  kBlue,
  kRed,
};

struct FeederReadings {
  frc::DriverStation::Alliance alliance;
  FeederBallState ball_state;

  bool is_running;  // is the currently feeder running?
};

struct FeederTarget {
  double speed;  // [-1, 1] where + is up and - is down
};

class FeederSubsystem : public frc846::Subsystem<FeederReadings, FeederTarget> {
 public:
  FeederSubsystem();

  // Speed when shooting.
  frc846::Pref<double> shoot_feed_speed_{*this, "shoot_feed_speed", 0.4};
  frc846::Pref<double> dumb_feed_speed_{*this, "dumb_feed_speed", 0.8};

  // Speed when reversing feeder
  frc846::Pref<double> reverse_feed_speed_{*this, "reverse_feed_speed", -0.5};

  // Speed when loading balls.
  frc846::Pref<double> load_feed_speed_{*this, "load_feed_speed", 0.2};

  // Whether or not to run auto load/ejection.
  frc846::Pref<bool> enable_auto_load_{*this, "enable_auto_load", true};

  FeederTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::Pref<double> blue_threshold_{*this, "blue_threshold", 0.29};
  frc846::Pref<double> red_threshold_{*this, "red_threshold", 0.38};
  frc846::Pref<double> ir_threshold_{*this, "ir_threshold", 5};

  frc846::Named color_sensor_named_{*this, "color_sensor"};
  frc846::Grapher<double> blue_graph_{color_sensor_named_, "blue"};
  frc846::Grapher<double> red_graph_{color_sensor_named_, "red"};
  frc846::Grapher<double> ir_graph_{color_sensor_named_, "ir"};

  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<double> target_speed_graph_{target_named_, "speed"};

  rev::CANSparkMax esc_{ports::feeder::kCANID,
                        rev::CANSparkMaxLowLevel::MotorType::kBrushless};

  frc846::Named esc_named_{*this, "esc"};

  frc846::motor::SparkMAXHelper esc_helper_{
      esc_named_,
      esc_,
      new frc846::motor::SparkMAXConfigHelper{
          esc_named_,
          {
              1.0,  /* peak_output */
              12_V, /* voltage_comp_saturation */

              60_A, /* current_limit */
          },
      },
      nullptr,
  };

  rev::ColorSensorV3 color_sensor_{frc::I2C::Port::kMXP};

  bool is_running_;

  FeederReadings GetNewReadings() override;
  void WriteToHardware(FeederTarget target) override;
};

using OptionalFeederSubsystem =
    frc846::OptionalSubsystem<FeederSubsystem, FeederReadings, FeederTarget>;

#endif  // ROBOT_SUBSYSTEMS_FEEDER_H_