#include "robot/funky_robot.h"

#include <frc/DSControlWord.h>
#include <frc/RobotController.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/button/Trigger.h>
#include <hal/Notifier.h>

#include "cameraserver/CameraServer.h"
#include "frc846/named.h"
#include "frc846/sendable_callback.h"
#include "frc846/wpilib/time.h"
#include "frc846/xbox.h"
#include "robot/autos/left_one_steal_two_ball_auto_command.h"
#include "robot/autos/left_two_ball_auto_command.h"
#include "robot/autos/right_five_ball_auto_command.h"
#include "robot/autos/right_two_ball_auto_command.h"
#include "robot/commands/climber_position_command.h"
#include "robot/commands/climber_spin_command.h"
#include "robot/commands/drive_command.h"
#include "robot/commands/feeder_command.h"
#include "robot/commands/follow_trajectory_command.h"
#include "robot/commands/intake_command.h"
#include "robot/commands/leds_command.h"
#include "robot/commands/load_feeder_command.h"
#include "robot/commands/optimal_spinup_command.h"
#include "robot/commands/smart_shoot_command.h"
#include "robot/commands/spinup_command.h"

FunkyRobot::FunkyRobot() : frc846::Named{"funky_robot"} {
  next_loop_time_ = frc846::wpilib::CurrentFPGATime();

  int32_t status = 0;
  notifier_ = HAL_InitializeNotifier(&status);
  FRC_CheckErrorStatus(status, "{}", "InitializeNotifier");

  HAL_SetNotifierName(notifier_, "FunkyRobot", &status);
}

FunkyRobot::~FunkyRobot() {
  int32_t status = 0;
  HAL_StopNotifier(notifier_, &status);
  HAL_CleanNotifier(notifier_, &status);
}

void FunkyRobot::StartCompetition() {
  // Silence warnings related to missing joystick
  // (Doesn't do anything when connected to FMS)
  frc::DriverStation::SilenceJoystickConnectionWarning(true);

  // Disable live window.
  frc::LiveWindow::DisableAllTelemetry();

  // Add dashboard buttons
  frc::SmartDashboard::PutData(
      "zero_modules", new frc846::SendableCallback(
                          [this] { container_.drivetrain_.ZeroModules(); }));
  frc::SmartDashboard::PutData(
      "zero_bearing", new frc846::SendableCallback(
                          [this] { container_.drivetrain_.ZeroBearing(); }));
  frc::SmartDashboard::PutData(
      "zero_odometry", new frc846::SendableCallback(
                           [this] { container_.drivetrain_.ZeroOdometry(); }));
  frc::SmartDashboard::PutData(
      "verify_hardware",
      new frc846::SendableCallback([this] { VerifyHardware(); }));
  frc::SmartDashboard::PutData("zero_climber",
                               new frc846::SendableCallback([this] {
                                 if (container_.climber_.Initialized()) {
                                   container_.climber_.subsystem()->Zero();
                                 }
                               }));

  // Add autos to dashboard
  auto_chooser_.SetDefaultOption("right_five_ball_auto",
                                 new RightFiveBallAutoCommand{container_});
  auto_chooser_.AddOption("right_two_ball_auto",
                          new RightTwoBallAutoCommand{container_});
  auto_chooser_.AddOption("left_two_ball_auto",
                          new LeftTwoBallAutoCommand{container_});
  auto_chooser_.AddOption("left_one_steal_two_ball_auto",
                          new LeftOneStealTwoBallAutoCommand{container_});

  frc::SmartDashboard::PutData(&auto_chooser_);

  // Enable camera stream
  // frc::CameraServer::StartAutomaticCapture();

  // Verify robot hardware
  VerifyHardware();

  // Set initial target for all subsystems to zero.
  for (auto subsystem : container_.all_subsystems_) {
    subsystem->SetTargetZero();
  }

  // Report to driver station that robot is ready
  Debug("\n********** Funky robot initialized **********\n");
  HAL_ObserveUserProgramStarting();

  for (;;) {
    next_loop_time_ += kPeriod;

    // Set new notifier time
    int32_t status = 0;
    HAL_UpdateNotifierAlarm(notifier_, next_loop_time_.to<uint64_t>(), &status);
    FRC_CheckErrorStatus(status, "{}", "UpdateNotifierAlarm");

    // Wait for notifier
    auto time = HAL_WaitForNotifierAlarm(notifier_, &status);
    FRC_CheckErrorStatus(status, "{}", "WaitForNotifierAlarm");

    if (time == 0 || status != 0) {
      break;
    }

    // Start loop timing
    auto loop_start_time = frc846::wpilib::CurrentFPGATime();

    // Get current control mode
    frc::DSControlWord word{};
    Mode mode = Mode::kNone;
    if (word.IsDisabled()) {
      HAL_ObserveUserProgramDisabled();
      mode = Mode::kDisabled;
    } else if (word.IsAutonomous()) {
      HAL_ObserveUserProgramAutonomous();
      mode = Mode::kAutonomous;
    } else if (word.IsTeleop()) {
      HAL_ObserveUserProgramTeleop();
      mode = Mode::kTeleop;
    } else if (word.IsTest()) {
      HAL_ObserveUserProgramTest();
      mode = Mode::kTest;
    }

    // If mode changed
    if (last_mode_ != mode) {
      if (mode == Mode::kDisabled) {
        // Clear command scheduler
        Debug("Clearing command scheduler");
        frc2::CommandScheduler::GetInstance().CancelAll();
        frc2::CommandScheduler::GetInstance().ClearButtons();
      } else if (mode == Mode::kAutonomous) {
        if (container_.climber_.Initialized()) {
          container_.climber_.subsystem()->Zero();
        }

        // Get and run selected auto command
        auto_command_ = auto_chooser_.GetSelected();

        if (auto_command_ != nullptr) {
          Debug("Running auto: {}", auto_command_->GetName());
          auto_command_->Schedule();
        } else {
          Error("Auto command null!");
        }
      } else if (mode == Mode::kTeleop) {
        // Cancel auto command and setup teleop defaults/triggers
        if (auto_command_ != nullptr) {
          Debug("Cancelling auto command");
          auto_command_->Cancel();
          auto_command_ = nullptr;
        }

        Debug("Setting up teleop default/triggers");
        container_.shooter_.SetTargetZero();
        InitTeleopDefaults();
        InitTeleopTriggers();
      }

      last_mode_ = mode;
    }

    // Update subsystem readings
    for (auto subsystem : container_.all_subsystems_) {
      subsystem->UpdateReadings();
    }

    // Tick command scheduler
    frc2::CommandScheduler::GetInstance().Run();

    // Update subsystem hardware
    for (auto subsystem : container_.all_subsystems_) {
      subsystem->UpdateHardware();
    }

    // Zero bearing when user button is pressed
    // Needs to work when disabled, so this is not in the command scheduler.
    if (frc::RobotController::GetUserButton()) {
      container_.drivetrain_.ZeroBearing();
    }

    // Update dashboards
    frc::SmartDashboard::UpdateValues();
    frc::Shuffleboard::Update();

    // Update graphs
    time_remaining_graph_.Graph(frc::DriverStation::GetMatchTime());

    warnings_graph_.Graph(frc846::Named::warn_count());
    errors_graph_.Graph(frc846::Named::error_count());

    can_usage_graph_.Graph(
        frc::RobotController::GetCANStatus().percentBusUtilization * 100);

    auto loop_time = frc846::wpilib::CurrentFPGATime() - loop_start_time;
    loop_time_graph_.Graph(frc846::wpilib::CurrentFPGATime() - loop_start_time);

    // Check loop time
    if (loop_time > kPeriod * 2) {
      Warn("Bad loop overrun: {} (loop period: {})",
           loop_time.convert<units::millisecond>(), kPeriod);
    }
  }
}

void FunkyRobot::EndCompetition() {
  Debug("\n********** Robot code ending **********\n");
}

void FunkyRobot::InitTeleopDefaults() {
  container_.drivetrain_.SetDefaultCommand(DriveCommand{container_});
  container_.leds_.SetDefaultCommand(LEDsCommand{container_});

  container_.feeder_.SetDefaultCommand(LoadFeederCommand{
      container_, container_.feeder_.subsystem()->enable_auto_load_});
}

void FunkyRobot::InitTeleopTriggers() {
  frc2::Trigger drivetrain_zero_bearing_trigger{
      [&] { return container_.driver_.readings().back_button; }};

  frc2::Trigger intake_trigger{
      [&] { return container_.driver_.readings().left_trigger; }};
  frc2::Trigger not_intake_trigger{
      [&] { return !container_.driver_.readings().left_trigger; }};

  frc2::Trigger feeder_trigger{
      [&] { return container_.operator_.readings().right_bumper; }};
  frc2::Trigger reverse_feeder_trigger{[&] {
    return !container_.operator_.readings().left_bumper &&
           container_.operator_.readings().a_button;
  }};

  frc2::Trigger smart_shoot_trigger{
      [&] { return container_.operator_.readings().right_trigger; }};

  frc2::Trigger reverse_intake_trigger{
      [&] { return container_.driver_.readings().a_button; }};

  frc2::Trigger shooter_rumble_trigger{
      [&] { return container_.shooter_.readings().is_ready; }};

  frc2::Trigger spinup_close_trigger{[&] {
    return !container_.operator_.readings().left_bumper &&
           container_.operator_.readings().pov == frc846::XboxPOV::kUp;
  }};
  frc2::Trigger spinup_mid_trigger{[&] {
    return !container_.operator_.readings().left_bumper &&
           container_.operator_.readings().pov == frc846::XboxPOV::kLeft;
  }};
  frc2::Trigger spinup_far_trigger{[&] {
    return !container_.operator_.readings().left_bumper &&
           container_.operator_.readings().pov == frc846::XboxPOV::kDown;
  }};
  frc2::Trigger optimal_spinup_trigger{[&] {
    return !container_.operator_.readings().left_bumper &&
           container_.operator_.readings().pov == frc846::XboxPOV::kRight;
  }};

  frc2::Trigger climber_intake_out_trigger{
      [&] { return container_.operator_.readings().left_bumper; }};

  frc2::Trigger climber_manual_climb_trigger{[&] {
    return container_.operator_.readings().left_bumper &&
           container_.operator_.readings().back_button;
  }};
  frc2::Trigger climber_manual_swing_trigger{[&] {
    return container_.operator_.readings().left_bumper &&
           container_.operator_.readings().start_button;
  }};

  frc2::Trigger climber_setpoint_stow_trigger{[&] {
    return container_.operator_.readings().left_bumper &&
           container_.operator_.readings().y_button;
  }};
  frc2::Trigger climber_setpoint_align_trigger{[&] {
    return container_.operator_.readings().left_bumper &&
           container_.operator_.readings().x_button;
  }};
  frc2::Trigger climber_setpoint_climb_high_trigger{[&] {
    return container_.operator_.readings().left_bumper &&
           container_.operator_.readings().pov == frc846::XboxPOV::kUp;
  }};
  frc2::Trigger climber_setpoint_swing_high_trigger{[&] {
    return container_.operator_.readings().left_bumper &&
           container_.operator_.readings().pov == frc846::XboxPOV::kRight;
  }};
  frc2::Trigger climber_setpoint_climb_traversal_trigger{[&] {
    return container_.operator_.readings().left_bumper &&
           container_.operator_.readings().pov == frc846::XboxPOV::kDown;
  }};
  frc2::Trigger climber_setpoint_swing_traversal_trigger{[&] {
    return container_.operator_.readings().left_bumper &&
           container_.operator_.readings().pov == frc846::XboxPOV::kLeft;
  }};

  drivetrain_zero_bearing_trigger.WhenActive(
      [this] { container_.drivetrain_.ZeroBearing(); });

  intake_trigger.WhileActiveOnce(IntakeCommand{container_});
  not_intake_trigger.WhileActiveOnce(frc2::FunctionalCommand{
      [&] {
        container_.intake_.SetTarget(
            {false, container_.intake_.subsystem()->intake_speed_.value()});
      },
      [] {},
      [&](bool) { container_.intake_.SetTargetZero(); },
      [] { return false; },
      {&container_.intake_},
  }
                                         .WithTimeout(1_s));

  feeder_trigger.WhileActiveOnce(FeederCommand{
      container_, container_.feeder_.subsystem()->dumb_feed_speed_});
  reverse_feeder_trigger.WhileActiveOnce(FeederCommand{
      container_, container_.feeder_.subsystem()->reverse_feed_speed_});
  smart_shoot_trigger.WhileActiveOnce(SmartShootCommand{container_});

  reverse_intake_trigger.WhileActiveOnce(IntakeCommand{container_, true});

  if (container_.shooter_.Initialized()) {
    auto shooter = container_.shooter_.subsystem();

    spinup_close_trigger.WhileActiveOnce(
        SpinupCommand{container_, shooter->preset_close_});
    spinup_mid_trigger.WhileActiveOnce(
        SpinupCommand{container_, shooter->preset_mid_});
    spinup_far_trigger.WhileActiveOnce(
        SpinupCommand{container_, shooter->preset_far_});
    optimal_spinup_trigger.WhileActiveOnce(OptimalSpinupCommand{container_});
  }

  shooter_rumble_trigger.WhileActiveOnce(frc2::FunctionalCommand{
      [&] { container_.operator_.SetTarget({true}); },
      [] {},
      [&](bool) { container_.operator_.SetTargetZero(); },
      [] { return false; },
      {&container_.intake_},
  });

  if (container_.climber_.Initialized()) {
    auto climber = container_.climber_.subsystem();

    climber_intake_out_trigger.WhileActiveOnce(
        frc2::FunctionalCommand{
            [&] {
              container_.intake_.SetTarget({true, 0.0});
            },
            [] {},
            [&](bool) { container_.intake_.SetTargetZero(); },
            [] { return false; },
            {&container_.intake_},
        },
        false /* interruptable */);

    climber_manual_climb_trigger.WhileActiveOnce(
        ClimberSpinCommand{container_, climber->manual_climb_speed_});
    climber_manual_swing_trigger.WhileActiveOnce(
        ClimberSpinCommand{container_, climber->manual_swing_speed_});

    climber_setpoint_stow_trigger.WhileActiveOnce(ClimberPositionCommand{
        container_,
        climber->setpoints_pre_stow_,
        ClimberPositionCommand::Direction::kEither,
    });
    climber_setpoint_align_trigger.WhileActiveOnce(ClimberPositionCommand{
        container_,
        climber->setpoints_align_,
        ClimberPositionCommand::Direction::kEither,
    });

    climber_setpoint_climb_high_trigger.WhileActiveOnce(ClimberPositionCommand{
        container_,
        climber->setpoints_climb_high_,
        ClimberPositionCommand::Direction::kClimb,
    });
    climber_setpoint_swing_high_trigger.WhileActiveOnce(ClimberPositionCommand{
        container_,
        climber->setpoints_swing_high_,
        ClimberPositionCommand::Direction::kSwing,
    });
    climber_setpoint_climb_traversal_trigger.WhileActiveOnce(
        ClimberPositionCommand{
            container_,
            climber->setpoints_climb_traversal_,
            ClimberPositionCommand::Direction::kClimb,
        });
    climber_setpoint_swing_traversal_trigger.WhileActiveOnce(
        ClimberPositionCommand{
            container_,
            climber->setpoints_swing_traversal_,
            ClimberPositionCommand::Direction::kSwing,
        });
  }
}

void FunkyRobot::VerifyHardware() {
  Debug("Verifying hardware...");
  for (auto subsystem : container_.all_subsystems_) {
    bool ok = subsystem->VerifyHardware();
    if (!ok) {
      subsystem->Error("Failed hardware verification!!");
    }
  }
  Debug("Done verifying hardware");
}