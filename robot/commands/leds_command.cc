#include "robot/commands/leds_command.h"

LEDsCommand::LEDsCommand(RobotContainer& container)
    : leds_(container.leds_),
      limelight_(container.limelight_),
      driver_(container.driver_) {
  AddRequirements({&leds_});
  SetName("leds_command");
}

void LEDsCommand::Execute() {
  if (driver_.readings().right_trigger) {
    if (limelight_.readings().target_exists) {
      leds_.SetTarget(LEDsTarget::kTargetFound);
    } else {
      leds_.SetTarget(LEDsTarget::kNoTargetFound);
    }
  } else {
    leds_.SetTarget(LEDsTarget::kRainbow);
  }
}