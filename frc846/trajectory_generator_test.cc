#include "frc846/trajectory_generator.h"

#include <gtest/gtest.h>

#include <stdexcept>

#include "frc846/math.h"

TEST(TrajectoryGeneratorTest, InterpolatePointsSmallCut) {
  auto points = frc846::InterpolatePoints(
      {8_ft, 4_ft}, {8_ft + 4_ft * 5, 4_ft + 3_ft * 5}, 5_ft);

  std::vector<frc846::Vector2D<units::foot_t>> expected{
      {8_ft + 4_ft * 0, 4_ft + 3_ft * 0}, {8_ft + 4_ft * 1, 4_ft + 3_ft * 1},
      {8_ft + 4_ft * 2, 4_ft + 3_ft * 2}, {8_ft + 4_ft * 3, 4_ft + 3_ft * 3},
      {8_ft + 4_ft * 4, 4_ft + 3_ft * 4},
  };

  ASSERT_EQ(points.size(), expected.size());
  for (unsigned int i = 0; i < expected.size(); i++) {
    EXPECT_EQ(points[i].x, expected[i].x);
    EXPECT_EQ(points[i].y, expected[i].y);
  }
}

TEST(TrajectoryGeneratorTest, InterpolatePointsLargeCut) {
  auto points = frc846::InterpolatePoints({8_ft, 4_ft}, {4_ft, 8_ft}, 1000_ft);

  frc846::Vector2D<units::foot_t> expected{8_ft, 4_ft};

  EXPECT_EQ(points.size(), 1);
  EXPECT_EQ(points[0].x, expected.x);
  EXPECT_EQ(points[0].y, expected.y);
}
