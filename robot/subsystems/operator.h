#ifndef ROBOT_SUBSYSTEMS_OPERATOR_H_
#define ROBOT_SUBSYSTEMS_OPERATOR_H_

#include "frc846/grapher.h"
#include "frc846/math.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"
#include "frc846/xbox.h"
#include "robot/ports.h"

using OperatorReadings = frc846::XboxReadings;

struct OperatorTarget {
  bool rumble;
};

class OperatorSubsystem
    : public frc846::Subsystem<OperatorReadings, OperatorTarget> {
 public:
  OperatorSubsystem();

  OperatorTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  // Left/right trigger button threshold.
  frc846::Pref<double> trigger_threshold_{*this, "trigger_threshold", 0.3};

  // Rumble vibration strength.
  frc846::Pref<double> rumble_strength_{*this, "rumble_strength", 1.0};

  frc::XboxController xbox_{ports::operator_::kXbox_DSPort};

  frc846::Named target_named_{*this, "target"};
  frc846::Grapher<bool> target_rumble_graph_{target_named_, "rumble"};

  OperatorReadings GetNewReadings() override;

  void WriteToHardware(OperatorTarget target) override;
};

#endif  // ROBOT_SUBSYSTEMS_Dawg_H_