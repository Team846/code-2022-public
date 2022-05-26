#include "robot/subsystems/limelight.h"

#include <vector>

#include "robot/field.h"

LimelightSubsystem::LimelightSubsystem()
    : Subsystem<LimelightReadings, LimelightTarget>("limelight") {}

LimelightReadings LimelightSubsystem::GetNewReadings() {
  LimelightReadings readings;

  // Throws error if limelight and toggle aren't in the same state
  // TODO no way to detect from limelight which mode it's running
  if (toggle_ellipse_.value()) {
    std::vector<double> llpython =
        table_->GetEntry("llpython")
            .GetDoubleArray({0.0, 0.0, 0.0});  // tv, tx, ty

    if (llpython.size() < 3) {
      Error("Wrong llpython length from limelight! len: {}", llpython.size());
      readings.target_exists = false;
    } else {
      readings.target_exists = llpython[0] != 0.0;
      readings.tx = units::degree_t(llpython[1]);
      readings.ty = units::degree_t(llpython[2]);
    }

    // Post tuning values to python script
    table_->PutNumberArray(
        "llrobot",
        std::vector<double>{low_h_.value(), low_s_.value(), low_v_.value(),
                            high_h_.value(), high_s_.value(), high_v_.value()});

  } else {
    readings.tx = units::degree_t(table_->GetNumber("tx", 0.0));
    readings.ty = units::degree_t(table_->GetNumber("ty", 0.0));
    readings.tl = units::millisecond_t(table_->GetNumber("tl", 0.0));
    readings.target_exists = table_->GetNumber("tv", 0.0) != 0.0;
  }

  units::foot_t vertical_distance =
      field::hub::kTopHeight - mounting_height_.value();

  // horizontal distance to target
  readings.hub_distance =
      vertical_distance /
          units::math::tan(readings.ty + mounting_angle_.value()) +
      (field::hub::kOuterDiameter / 2);

  hub_distance_graph_.Graph(readings.hub_distance);
  target_exists_graph_.Graph(readings.target_exists);
  tx_graph_.Graph(readings.tx);
  ty_graph_.Graph(readings.ty);

  return readings;
}

bool LimelightSubsystem::VerifyHardware() { return true; }

void LimelightSubsystem::WriteToHardware(LimelightTarget target) {
  table_->PutNumber("snapshot", target.take_snapshot ? 1 : 0);
}

LimelightTarget LimelightSubsystem::ZeroTarget() const { return {}; }