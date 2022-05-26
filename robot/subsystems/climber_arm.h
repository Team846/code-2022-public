#ifndef ROBOT_SUBSYSTEMS_CLIMBER_ARM_H_
#define ROBOT_SUBSYSTEMS_CLIMBER_ARM_H_

#include <rev/CANSparkMax.h>
#include <units/angle.h>

#include <variant>

#include "frc846/grapher.h"
#include "frc846/motor/converter.h"
#include "frc846/motor/helper.h"
#include "frc846/subsystem.h"

struct ClimberArmReadings {
  // Position relative to stow position
  units::degree_t position;
};

struct ClimberArmTarget {
  // [-1, 1] where + is climbing and - is swinging
  double speed;
};

// Individual climber arm subsystem. `ClimberSubsystem` contains two.
class ClimberArmSubsystem
    : public frc846::Subsystem<ClimberArmReadings, ClimberArmTarget> {
 public:
  ClimberArmSubsystem(const frc846::Named& climber, std::string location,
                      bool inverted,
                      frc846::motor::SparkMAXConfigHelper* esc_config_helper,
                      int leader_can_id, int follower_can_id);

  // Zero the position to stow.
  void Zero();

  ClimberArmTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  // Is the arm inverted?
  bool inverted_;

  frc846::Grapher<units::degree_t> position_graph_{*this, "position"};

  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<double> target_speed_graph_{target_named_, "speed"};

  frc846::motor::Converter<units::degree_t> arm_converter_{
      frc846::motor::kSparkMAXPeriod,
      frc846::motor::kSparkMAXSensorTicks,
      // 2x 3:1 MAXPlanetary, 20:74 gearstage, 15:60 sprocket
      1_tr / (3.0 * 3.0) * (20.0 / 74.0) * (15.0 / 60.0),
  };

  rev::CANSparkMax leader_esc_;
  rev::CANSparkMax follower_esc_;

  frc846::Named esc_named_{*this, "esc"};
  frc846::motor::SparkMAXHelper leader_esc_helper_;
  frc846::motor::SparkMAXHelper follower_esc_helper_;

  ClimberArmReadings GetNewReadings() override;

  void WriteToHardware(ClimberArmTarget target) override;
};

#endif  // ROBOT_SUBSYSTEMS_CLIMBER_ARM_H_