#ifndef ROBOT_FUNKY_ROBOT_H_
#define ROBOT_FUNKY_ROBOT_H_

#include <frc/Compressor.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <hal/Types.h>
#include <units/time.h>

#include "frc846/pref.h"
#include "robot/subsystems/robot_container.h"

enum Mode { kNone, kDisabled, kAutonomous, kTeleop, kTest };

class FunkyRobot : public frc::RobotBase, public frc846::Named {
 public:
  static constexpr auto kPeriod = 20_ms;  // 50hz

  FunkyRobot();

  ~FunkyRobot() override;

  void StartCompetition() override;
  void EndCompetition() override;

  void InitTeleopDefaults();
  void InitTeleopTriggers();

  void VerifyHardware();

 private:
  hal::Handle<HAL_NotifierHandle> notifier_;
  units::microsecond_t next_loop_time_;

  Mode last_mode_;

 private:
  RobotContainer container_;

  frc::Compressor compressor_{frc::PneumaticsModuleType::CTREPCM};

  frc846::Grapher<int> time_remaining_graph_{*this, "time"};

  frc846::Grapher<int> warnings_graph_{*this, "warnings"};
  frc846::Grapher<int> errors_graph_{*this, "errors"};

  frc846::Grapher<double> can_usage_graph_{*this, "CAN_usage"};
  frc846::Grapher<units::millisecond_t> loop_time_graph_{*this, "loop_time"};

  frc::SendableChooser<frc2::Command*> auto_chooser_;
  frc2::Command* auto_command_ = nullptr;
};

#endif  // ROBOT_FUNKY_ROBOT_H_