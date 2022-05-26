#ifndef ROBOT_SUBSYSTEMS_INTAKE_H_
#define ROBOT_SUBSYSTEMS_INTAKE_H_

#include <frc/Solenoid.h>

#include "frc846/grapher.h"
#include "frc846/motor/helper.h"
#include "frc846/named.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "robot/ports.h"

struct IntakeReadings {};

struct IntakeTarget {
  bool is_extended;
  double speed;  // [-1.0, 1.0] where + is intaking direction
};

class IntakeSubsystem : public frc846::Subsystem<IntakeReadings, IntakeTarget> {
 public:
  IntakeSubsystem();

  frc846::Pref<double> intake_speed_{*this, "intake_speed", 0.65};

  IntakeTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<bool> target_is_extended_graph_{target_named_, "is_extended"};
  frc846::Grapher<double> target_speed_graph_{target_named_, "speed"};

  rev::CANSparkMax esc_{ports::intake::kCANID,
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

  frc::Solenoid intake_solenoid_{frc::PneumaticsModuleType::CTREPCM,
                                 ports::intake::kPHPort};

  IntakeReadings GetNewReadings() override;

  void WriteToHardware(IntakeTarget target) override;
};

using OptionalIntakeSubsystem =
    frc846::OptionalSubsystem<IntakeSubsystem, IntakeReadings, IntakeTarget>;

#endif  // ROBOT_SUBSYSTEMS_INTAKE_H_