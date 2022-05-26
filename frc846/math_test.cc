#include "frc846/math.h"

#include <gtest/gtest.h>

TEST(MathTest, Circumference) {
  EXPECT_EQ(frc846::Circumference(0.5_ft), units::constants::pi * 1_ft);
}

TEST(MathTest, Vector2DMagnitude) {
  // Q1
  EXPECT_EQ((frc846::Vector2D<units::inch_t>{+3_in, +4_in}).Magnitude(), 5_in);
  // Q2
  EXPECT_EQ((frc846::Vector2D<units::inch_t>{-3_in, +4_in}).Magnitude(), 5_in);
  // Q3
  EXPECT_EQ((frc846::Vector2D<units::inch_t>{-3_in, -4_in}).Magnitude(), 5_in);
  // Q4
  EXPECT_EQ((frc846::Vector2D<units::inch_t>{+3_in, -4_in}).Magnitude(), 5_in);
}

TEST(MathTest, Vector2DBearing) {
  // Q1
  EXPECT_EQ(
      (frc846::Vector2D<units::inch_t>{+1_in, +std::sqrt(3) * 1_in}).Bearing(),
      30_deg);
  // Q2
  EXPECT_EQ(
      (frc846::Vector2D<units::inch_t>{-1_in, +std::sqrt(3) * 1_in}).Bearing(),
      -30_deg);
  // Q3
  EXPECT_EQ(
      (frc846::Vector2D<units::inch_t>{-1_in, -std::sqrt(3) * 1_in}).Bearing(),
      -150_deg);
  // Q4
  EXPECT_EQ(
      (frc846::Vector2D<units::inch_t>{+1_in, -std::sqrt(3) * 1_in}).Bearing(),
      150_deg);
}

TEST(MathTest, Vector2DRotate) {
  frc846::Vector2D<units::inch_t> v{8_in, 4_in};

  EXPECT_EQ(v.Rotate(+90_deg), (frc846::Vector2D<units::inch_t>{+4_in, -8_in}));
  EXPECT_EQ(v.Rotate(-90_deg), (frc846::Vector2D<units::inch_t>{-4_in, +8_in}));
}