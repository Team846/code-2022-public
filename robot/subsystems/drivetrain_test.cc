#include "robot/subsystems/drivetrain.h"

#include <gtest/gtest.h>

// Dimensions of the test (non-square) drivetrain
static constexpr units::inch_t kWidth = 15_in;
static constexpr units::inch_t kHeight = 10_in;
static const units::inch_t kRadius = units::math::sqrt(
    units::math::pow<2>(kWidth) + units::math::pow<2>(kHeight));

// Swerve control test case.
struct TestCase {
  // Drivetrain translation input.
  frc846::Vector2D<units::feet_per_second_t> translation;

  // Drivetrain rotation input.
  units::degrees_per_second_t rotation_speed;

  // Expected targets for each module.
  SwerveModuleTarget expected_targets[DrivetrainSubsystem::kModuleCount];
};

// Test the given swerve control test case by asserting that each module's
// calculated output is equal to the expected targets.
#define EXPECT_SWERVE_CONTROL(c)                                               \
  do {                                                                         \
    auto targets = DrivetrainSubsystem::SwerveControl(                         \
        c.translation, c.rotation_speed, kWidth, kHeight, kRadius, 10e9_fps);  \
                                                                               \
    for (int i = 0; i < DrivetrainSubsystem::kModuleCount; ++i) {              \
      SwerveModuleTarget target{targets[i].Magnitude(), targets[i].Bearing()}; \
                                                                               \
      EXPECT_FLOAT_EQ(target.speed.to<double>(),                               \
                      c.expected_targets[i].speed.to<double>());               \
      EXPECT_FLOAT_EQ(target.direction.to<double>(),                           \
                      c.expected_targets[i].direction.to<double>());           \
    }                                                                          \
  } while (0)

// Test swerve control translation (up/down/left/right and diagonals).
TEST(DrivetrainTest, SwerveControlTranslation) {
  const units::feet_per_second_t s = 3_fps;  // Test speed
  const units::feet_per_second_t sr2 =
      s * std::sqrt(2);  // Test speed when going diagonal (s√2)

  std::vector<TestCase> cases{
      {
          {0_fps, +s},  // Going up at s speed
          0_deg_per_s,  // No rotation
          {
              // All modules should point at 0° at s speed
              {s, 0_deg},
              {s, 0_deg},
              {s, 0_deg},
              {s, 0_deg},
          },
      },
      {
          {0_fps, -s},  // Going down at s speed
          0_deg_per_s,  // No rotation
          {
              // All modules should point at 180° at s speed
              {s, 180_deg},
              {s, 180_deg},
              {s, 180_deg},
              {s, 180_deg},
          },
      },

      {
          {-s, 0_fps},  // Going left at s speed
          0_deg_per_s,  // No rotation
          {
              // All modules should point at -90° at s speed
              {s, -90_deg},
              {s, -90_deg},
              {s, -90_deg},
              {s, -90_deg},
          },
      },
      {
          {+s, 0_fps},  // Going right at s speed
          0_deg_per_s,  // No rotation
          {
              // All modules should point at 90° at s speed
              {s, 90_deg},
              {s, 90_deg},
              {s, 90_deg},
              {s, 90_deg},
          },
      },
      {
          {+s, +s},     // Going up/right at s√2 speed
          0_deg_per_s,  // No rotation
          {
              // All modules should point at 45° at s√2 speed.
              {sr2, 45_deg},
              {sr2, 45_deg},
              {sr2, 45_deg},
              {sr2, 45_deg},
          },
      },
      {
          {+s, -s},     // Going down/right at s√2 speed
          0_deg_per_s,  // No rotation
          {
              // All modules should point at 135° at s√2 speed
              {sr2, 135_deg},
              {sr2, 135_deg},
              {sr2, 135_deg},
              {sr2, 135_deg},
          },
      },
      {
          {-s, -s},     // Going down/left at s√2 speed
          0_deg_per_s,  // No rotation
          {
              // All modules should point at -135° at s√2 speed
              {sr2, -135_deg},
              {sr2, -135_deg},
              {sr2, -135_deg},
              {sr2, -135_deg},
          },
      },
      {
          {-s, +s},     // Going up/left at s√2 speed
          0_deg_per_s,  // No rotation
          {
              // All modules should point at -45° at s√2 speed
              {sr2, -45_deg},
              {sr2, -45_deg},
              {sr2, -45_deg},
              {sr2, -45_deg},
          },
      },
  };

  for (const TestCase& c : cases) {
    EXPECT_SWERVE_CONTROL(c);
  }
}

// Test swerve control rotation (clockwise, counterclockwise).
TEST(DrivetrainTest, SwerveControlRotation) {
  const units::feet_per_second_t s = 3_fps;  // Test speed

  const units::degree_t t = units::math::atan2(kHeight / 2, kWidth / 2);

  std::vector<TestCase> cases{
      {
          {0_fps, 0_fps},        // No translation
          +s / kRadius * 1_rad,  // Rotating clockwise at r speed
          {
              {s, t},            // Pointed at atan(h/w)  at r speed
              {s, 180_deg - t},  // Pointed at 180 - atan(h/w) at r speed
              {s, -t},           // Pointed at -atan(h/w)  at r speed
              {s, t - 180_deg},  // Pointed at atan(h/w) - 180 at r speed
          },
      },
      {
          {0_fps, 0_fps},        // No translation
          -s / kRadius * 1_rad,  // Rotating counterclockwise at r speed
          {
              {s, t - 180_deg},  // Pointed at atan(h/w) - 180 at r speed
              {s, -t},           // Pointed at -atan(h/w)  at r speed
              {s, 180_deg - t},  // Pointed at 180 - atan(h/w) at r speed
              {s, t},            // Pointed at atan(h/w)  at r speed
          },
      },
  };

  for (const TestCase& c : cases) {
    EXPECT_SWERVE_CONTROL(c);
  }
}

// Test swerve control translation + rotation combinations.
TEST(DrivetrainTest, SwerveControlCombined) {
  const units::feet_per_second_t s = 3_fps;  // Test speed
  const units::feet_per_second_t sr2 =
      s * std::sqrt(2);  // Test speed when going diagonal (s√2)
  const units::degrees_per_second_t r =
      s / kRadius * 1_rad;  // Test rotation speed

  const units::degree_t t = units::math::atan2(kHeight / 2, kWidth / 2);

  // Add two swerve module targets together.
  auto add = [](SwerveModuleTarget t1, SwerveModuleTarget t2) {
    frc846::Vector2D<units::feet_per_second_t> sum =
        frc846::Vector2D<units::feet_per_second_t>{
            t1.speed * units::math::cos(90_deg - t1.direction),
            t1.speed * units::math::sin(90_deg - t1.direction),
        } +
        frc846::Vector2D<units::feet_per_second_t>{
            t2.speed * units::math::cos(90_deg - t2.direction),
            t2.speed * units::math::sin(90_deg - t2.direction),
        };

    return SwerveModuleTarget{sum.Magnitude(), sum.Bearing()};
  };

  std::vector<TestCase> cases{
      {
          {-s, +s},  // Going up/left at s√2 speed
          +r,        // Rotating clockwise at r speed
          {
              add({sr2, -45_deg}, {s, t}),
              add({sr2, -45_deg}, {s, 180_deg - t}),
              add({sr2, -45_deg}, {s, -t}),
              add({sr2, -45_deg}, {s, t - 180_deg}),
          },
      },
      {
          {+s, -s},  // Going down/right at s√2 speed
          -r,        // Rotating counterclockwise at s speed
          {
              add({sr2, 135_deg}, {s, t - 180_deg}),
              add({sr2, 135_deg}, {s, -t}),
              add({sr2, 135_deg}, {s, 180_deg - t}),
              add({sr2, 135_deg}, {s, t}),
          },
      },
  };

  for (const TestCase& c : cases) {
    EXPECT_SWERVE_CONTROL(c);
  }
}

// TODO test swerve control caps magnitudes.