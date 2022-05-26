#ifndef FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_
#define FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_

#include "frc846/pref.h"
#include "robot/subsystems/climber.h"
#include "robot/subsystems/driver.h"
#include "robot/subsystems/drivetrain.h"
#include "robot/subsystems/feeder.h"
#include "robot/subsystems/intake.h"
#include "robot/subsystems/leds.h"
#include "robot/subsystems/limelight.h"
#include "robot/subsystems/operator.h"
#include "robot/subsystems/shooter.h"

class RobotContainer : public frc846::Named {
 public:
  RobotContainer() : frc846::Named{"robot_container"} {}

 private:
  frc846::Pref<bool> init_intake_{*this, "init_intake", true};
  frc846::Pref<bool> init_feeder_{*this, "init_feeder", true};
  frc846::Pref<bool> init_shooter_{*this, "init_shooter", true};
  frc846::Pref<bool> init_climber_{*this, "init_climber", true};
  frc846::Pref<bool> init_limelight_{*this, "init_limelight", true};
  frc846::Pref<bool> init_leds_{*this, "init_leds", true};

 public:
  DriverSubsystem driver_;
  OperatorSubsystem operator_;

  DrivetrainSubsystem drivetrain_;

  OptionalIntakeSubsystem intake_{init_intake_.value(), "intake"};
  OptionalFeederSubsystem feeder_{init_feeder_.value(), "feeder"};
  OptionalShooterSubsystem shooter_{init_shooter_.value(), "shooter"};
  OptionalClimberSubsystem climber_{init_climber_.value(), "climber"};

  OptionalLimelightSubsystem limelight_{init_limelight_.value(), "limelight"};

  OptionalLEDsSubsystem leds_{init_leds_.value(), "leds"};

  std::vector<frc846::SubsystemBase*> all_subsystems_{
      &driver_,  &operator_,  &drivetrain_, &intake_, &feeder_,
      &shooter_, &climber_, &limelight_,  &leds_,
  };
};

#endif  // FRC846_SUBSYSTEMS_ROBOT_CONTAINER_H_