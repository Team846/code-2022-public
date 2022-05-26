#include "frc846/subsystem.h"

#include <gtest/gtest.h>

struct DummyReadings {
  double s;
};

struct DummyTarget {
  double b;
};

class DummySensor {
 public:
  void Set(double v) { value_ = v; }
  double Get() const { return value_; }

 private:
  double value_;
};

class DummyMotor {
 public:
  void Set(double v) {
    last_value_ = v;
    updated_ = true;
  }
  double GetLast() const { return last_value_; }

  bool IsUpdated() const { return updated_; }
  void ResetUpdated() { updated_ = false; }

 private:
  bool updated_;
  double last_value_;
};

class DummySubsystem : public frc846::Subsystem<DummyReadings, DummyTarget> {
 public:
  DummySubsystem(DummySensor& sensor, DummyMotor& motor)
      : frc846::Subsystem<DummyReadings, DummyTarget>{"dummy"},
        sensor_(sensor),
        motor_(motor) {}

  DummyTarget ZeroTarget() const override { return {0.0}; }

  bool VerifyHardware() override { return true; }

 private:
  DummySensor& sensor_;
  DummyMotor& motor_;

  DummyReadings GetNewReadings() override { return {sensor_.Get()}; }

  void WriteToHardware(DummyTarget target) override { motor_.Set(target.b); }
};

class SubsystemTest : public testing::Test {
 protected:
  DummySensor sensor_;
  DummyMotor motor_;
  DummySubsystem subsystem_{sensor_, motor_};
};

// Test that UpdateReadings gets the latest readings.
TEST_F(SubsystemTest, UpdateReadings) {
  for (auto s : {8, 4, 6}) {
    sensor_.Set(s);
    subsystem_.UpdateReadings();
    EXPECT_EQ(subsystem_.readings().s, s);
  }
}

// Test that UpdateHardware sets the target to the hardware with proper caching.
TEST_F(SubsystemTest, UpdateHardware) {
  subsystem_.SetTarget({8});
  subsystem_.UpdateHardware();
  EXPECT_TRUE(motor_.IsUpdated());
  EXPECT_EQ(motor_.GetLast(), 8);
  motor_.ResetUpdated();

  subsystem_.SetTarget({4});
  subsystem_.UpdateHardware();
  EXPECT_TRUE(motor_.IsUpdated());
  EXPECT_EQ(motor_.GetLast(), 4);
  motor_.ResetUpdated();
}