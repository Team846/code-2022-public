#include "robot/subsystems/swerve_module.h"

#include <gtest/gtest.h>

// Test the clock math to rotate the module to the nearest equivalent target
// angle.
TEST(SwerveModuleTest, NormalizedDirection) {
  // Normalized direction test case.
  struct TestCase {
    // Current module direction.
    units::degree_t current;

    // Target module direction.
    units::degree_t input_target;

    // Expected normalized module direction target.
    units::degree_t expected_target;

    // Should the module reverse drive directions?
    bool should_reverse_drive;
  };

  std::vector<TestCase> cases{
      // 0 to 1 (direct)
      {0_deg, 1_deg, 1_deg, false},
      // 0 to -1 (direct)
      {0_deg, -1_deg, -1_deg, false},
      // 0 to 89 (direct)
      {0_deg, 89_deg, 89_deg, false},
      // 0 to -89 (direct)
      {0_deg, -89_deg, -89_deg, false},
      // 0 to 181 (reverse and go to 1)
      {0_deg, 181_deg, 1_deg, true},
      // 0 to -181 (reverse and go to -1)
      {0_deg, -181_deg, -1_deg, true},
      // 0 to 271 (wrap CCW to -89)
      {0_deg, 271_deg, -89_deg, false},
      // 0 to -271 (wrap CW to 89)
      {0_deg, -271_deg, 89_deg, false},

      // 360 to 1 (add 360)
      {360_deg, 1_deg, 360_deg + 1_deg, false},
      // 360 to -1 (add 360)
      {360_deg, -1_deg, 360_deg - 1_deg, false},
      // 360 to 89 (add 360)
      {360_deg, 89_deg, 360_deg + 89_deg, false},
      // 360 to -89 (add 360)
      {360_deg, -89_deg, 360_deg - 89_deg, false},
      // 360 to 181 (add 360, reverse and go to 1)
      {360_deg, 181_deg, 360_deg + 1_deg, true},
      // 360 to -181 (add 360, reverse and go to -1)
      {360_deg, -181_deg, 360_deg - 1_deg, true},
      // 360 to 271 (add 360 and wrap CCW to -89)
      {360_deg, 271_deg, 360_deg - 89_deg, false},
      // 360 to -271 (add 360 and wrap CW to 89)
      {360_deg, -271_deg, 360_deg + 89_deg, false},

      // -180 to 1 (reverse, go to -179)
      {-180_deg, 1_deg, -180_deg + 1_deg, true},
      // -180 to -1 (reverse, go to -181)
      {-180_deg, -1_deg, -180_deg - 1_deg, true},
      // -180 to 89 (reverse, go to -91)
      {-180_deg, 89_deg, -91_deg, true},
      // -180 to -89 (reverse, go to -271)
      {-180_deg, -89_deg, -180_deg - 89_deg, true},
      // -180 to 181 (wrap CW to -179)
      {-180_deg, 181_deg, -180_deg + 1_deg, false},
      // -180 to -181 (direct)
      {-180_deg, -181_deg, -180_deg - 1_deg, false},
      // -180 to 271 (reverse)
      {-180_deg, 271_deg, -180_deg - 89_deg, true},
      // -180 to -271 (reverse)
      {-180_deg, -271_deg, -180_deg + 89_deg, true},
  };

  for (const TestCase& c : cases) {
    auto [normalized_angle, reverse_drive] =
        SwerveModuleSubsystem::NormalizedDirection(c.current, c.input_target);

    EXPECT_FLOAT_EQ(normalized_angle.to<double>(),
                    c.expected_target.to<double>());
    EXPECT_EQ(reverse_drive, c.should_reverse_drive);
  }
}