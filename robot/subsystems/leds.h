#ifndef ROBOT_SUBSYSTEMS_LEDS_H_
#define ROBOT_SUBSYSTEMS_LEDS_H_

#include <frc/AddressableLED.h>

#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "robot/ports.h"

struct LEDsReadings {};

enum LEDsTarget {
  kRainbow,        // idle state
  kTargetFound,    // green when aiming and target found
  kNoTargetFound,  // red when aiming and target not found
};

class LEDsSubsystem : public frc846::Subsystem<LEDsReadings, LEDsTarget> {
 public:
  LEDsSubsystem();

  LEDsTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  // Number of LEDs.
  static constexpr int kLength = 18;

  int first_pixel_hue_ = 0;

  std::array<frc::AddressableLED::LEDData, kLength> leds_buffer_;

  frc::AddressableLED leds_{ports::leds::kPWMPort};

  LEDsReadings GetNewReadings() override;

  void WriteToHardware(LEDsTarget target) override;
};

using OptionalLEDsSubsystem =
    frc846::OptionalSubsystem<LEDsSubsystem, LEDsReadings, LEDsTarget>;

#endif  // ROBOT_SUBSYSTEMS_LEDS_H_