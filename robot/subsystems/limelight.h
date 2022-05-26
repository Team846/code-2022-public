#ifndef ROBOT_SUBSYSTEMS_LIMELIGHT_H_
#define ROBOT_SUBSYSTEMS_LIMELIGHT_H_

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>

#include "frc846/grapher.h"
#include "frc846/named.h"
#include "frc846/pref.h"
#include "frc846/subsystem.h"

struct LimelightReadings {
  bool target_exists;  // same as tv

  units::degree_t tx;
  units::degree_t ty;
  units::second_t tl;

  units::foot_t hub_distance;  // distance to the goal according to ty
};

struct LimelightTarget {
  bool take_snapshot;  // whether or not to take a snapshot. (only takes
                       // snapshot on rising edge)
};

class LimelightSubsystem
    : public frc846::Subsystem<LimelightReadings, LimelightTarget> {
 public:
  LimelightSubsystem();

  frc846::Pref<units::inch_t> mounting_height_{*this, "mounting_height",
                                               38.545_in};
  frc846::Pref<units::degree_t> mounting_angle_{*this, "mounting_angle",
                                                25_deg};

  LimelightTarget ZeroTarget() const override;

  bool VerifyHardware() override;

 private:
  frc846::Grapher<units::foot_t> hub_distance_graph_{*this, "hub_distance"};

  frc846::Grapher<bool> target_exists_graph_{*this, "target_exists"};
  frc846::Grapher<units::degree_t> tx_graph_{*this, "tx"};
  frc846::Grapher<units::degree_t> ty_graph_{*this, "ty"};

  // Tuning values for custom ellipse pipline.
  frc846::Pref<double> low_h_{*this, "low_h", 60};
  frc846::Pref<double> low_s_{*this, "low_s", 168};
  frc846::Pref<double> low_v_{*this, "low_v", 118};

  frc846::Pref<double> high_h_{*this, "high_h", 85};
  frc846::Pref<double> high_s_{*this, "high_s", 255};
  frc846::Pref<double> high_v_{*this, "high_v", 255};

  // Whether to use custom ellipse python pipeline or regular limelight
  // pipeline.
  frc846::Pref<bool> toggle_ellipse_{*this, "toggle_ellipse", true};

  std::shared_ptr<nt::NetworkTable> table_ =
      nt::NetworkTableInstance::GetDefault().GetTable("limelight");

  LimelightReadings GetNewReadings() override;

  void WriteToHardware(LimelightTarget target) override;
};

using OptionalLimelightSubsystem =
    frc846::OptionalSubsystem<LimelightSubsystem, LimelightReadings,
                              LimelightTarget>;

#endif  // ROBOT_SUBSYSTEMS_LIMELIGHT_H_