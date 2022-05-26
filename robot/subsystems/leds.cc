#include "robot/subsystems/leds.h"

LEDsSubsystem::LEDsSubsystem()
    : frc846::Subsystem<LEDsReadings, LEDsTarget>("leds") {
  leds_.SetLength(kLength);
  leds_.SetData(leds_buffer_);
  leds_.Start();
}

LEDsTarget LEDsSubsystem::ZeroTarget() const {
  LEDsTarget target = ::kRainbow;
  return target;
}

bool LEDsSubsystem::VerifyHardware() { return true; }

LEDsReadings LEDsSubsystem::GetNewReadings() { return {}; }

void LEDsSubsystem::WriteToHardware(LEDsTarget target) {
  switch (target) {
    case LEDsTarget::kRainbow:
      for (int i = 0; i < kLength; i++) {
        const auto pixelHue = (first_pixel_hue_ + (i * 180 / kLength)) % 180;
        leds_buffer_[i].SetHSV(pixelHue, 255, 128);
      }
      first_pixel_hue_ += 3;
      first_pixel_hue_ %= 180;

      break;
    case LEDsTarget::kTargetFound:
      for (int i = 0; i < kLength; i++) {
        leds_buffer_[i].SetRGB(0, 255, 0);
      }

      break;
    case LEDsTarget::kNoTargetFound:
      for (int i = 0; i < kLength; i++) {
        leds_buffer_[i].SetRGB(255, 0, 0);
      }

      break;
  }

  leds_.SetData(leds_buffer_);
}
