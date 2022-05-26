#ifndef FRC846_SUBSYSTEM_H_
#define FRC846_SUBSYSTEM_H_

#include <frc2/command/InstantCommand.h>
#include <frc2/command/PerpetualCommand.h>
#include <frc2/command/SubsystemBase.h>

#include "frc846/named.h"

namespace frc846 {

#define FRC846_VERIFY(expr, ok, fail_msg)    \
  do {                                       \
    if (!(expr)) {                           \
      ok = false;                            \
      Error("HARDWARE ERROR: {}", fail_msg); \
    }                                        \
  } while (0)

// Non-templated subsystem base class.
class SubsystemBase : public Named {
 public:
  SubsystemBase(std::string_view name) : Named{name} {}
  SubsystemBase(const Named& parent, std::string_view name)
      : Named{parent, name} {}

  virtual ~SubsystemBase() = default;

  // Get the latest readings from the subsystem.
  virtual void UpdateReadings() = 0;

  // Write the latest target to the subsystem hardware.
  virtual void UpdateHardware() = 0;

  // Verify the subsystem hardware status.
  virtual bool VerifyHardware() = 0;

  // Set the subsystem target to its zero state.s
  virtual void SetTargetZero() = 0;
};

// Base class for robot subsystems.
template <class Readings, class Target>
class Subsystem : public frc2::SubsystemBase, public SubsystemBase {
 public:
  // Construct a new subsystem.
  explicit Subsystem(std::string_view name) : frc846::SubsystemBase{name} {
    Init();
  }

  // Construct a subsystem as a child of another subsystem.
  Subsystem(const Named& parent, std::string_view name)
      : frc846::SubsystemBase{parent, name} {
    Init();
  }

  Subsystem(const Subsystem&) = delete;
  Subsystem& operator=(const Subsystem&) = delete;

  virtual ~Subsystem() = default;

 private:
  // Common constructor.
  void Init() {
    SetName(name());
    Debug("Initializing");
  }

 public:
  // Get the zero state target.
  virtual Target ZeroTarget() const = 0;

  // Fetches new readings and update subsystem readings state.
  void UpdateReadings() override { readings_ = GetNewReadings(); }

  // Writes to subsystem hardware with the latest target output.
  void UpdateHardware() override { WriteToHardware(target_); }

  // Verify hardware status.
  virtual bool VerifyHardware() override = 0;

  // Get the latest readings.
  Readings readings() const { return readings_; };

  // Set the subystem target state.
  void SetTarget(Target target) { target_ = target; }

  // Set the subsystem to its zero state.
  void SetTargetZero() override { target_ = ZeroTarget(); }

 private:
  Readings readings_;
  Target target_;

  // Fetches and return new readings.
  virtual Readings GetNewReadings() = 0;

  // Writes output to hardware.
  virtual void WriteToHardware(Target target) = 0;
};

// Wraps a subsystem to make it optional.
//
// Optional subsystem will create a `init` preference to create the subsystem or
// not. If the subsystem is not initialized, `UpdateReadings` and
// `UpdateHardware` will return 0 and do nothing respectively.
//
// TODO implementation is a little messy, `subsystem()` calls everywhere.
template <class S, class Readings, class Target>
class OptionalSubsystem : public Subsystem<Readings, Target> {
 public:
  OptionalSubsystem(bool init, std::string_view name)
      : Subsystem<Readings, Target>{fmt::format("optional_{}", name)} {
    if (init) {
      subsystem_ = new S{};
    }
  }

  ~OptionalSubsystem() {
    if (subsystem_) {
      delete subsystem_;
    }
  }

  // Access the underlying subsystem if you need to access a property or method.
  //
  // TODO this is dangerous, returns null pointer if not intialized.
  S* subsystem() const { return subsystem_; }

  // Check if the subsystem is initialized.
  bool Initialized() const { return subsystem_; }

  bool VerifyHardware() override {
    if (subsystem_) {
      return subsystem_->VerifyHardware();
    }
    return true;
  }

  Target ZeroTarget() const override {
    if (subsystem_) {
      return subsystem_->ZeroTarget();
    } else {
      return Target{};
    }
  }

 private:
  S* subsystem_ = nullptr;

  Readings GetNewReadings() override {
    if (subsystem_) {
      subsystem_->UpdateReadings();
      return subsystem_->readings();
    } else {
      return Readings{};
    }
  }

  void WriteToHardware(Target target) override {
    if (subsystem_) {
      subsystem_->SetTarget(target);
      subsystem_->UpdateHardware();
    }
  }
};

}  // namespace frc846

#endif  // FRC846_SUBSYSTEM_H_