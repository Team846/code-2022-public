#include "frc846/grapher.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <gtest/gtest.h>
#include <units/length.h>

TEST(GrapherTest, Graph) {
  frc846::Named named{"root"};
  frc846::Grapher<units::inch_t> g1{named, "g1"};
  frc846::Grapher<bool> g2{named, "g2"};
  frc846::Grapher<double> g3{named, "g3"};
  frc846::Grapher<int> g4{named, "g4"};
  frc846::Grapher<std::string> g5{named, "g5"};

  g1.Graph(8.46_in);
  g2.Graph(true);
  g3.Graph(8.46);
  g4.Graph(846);
  g5.Graph("846");

  EXPECT_EQ(frc::SmartDashboard::GetNumber("root/g1 (in)", 0.0), 8.46);
  EXPECT_EQ(frc::SmartDashboard::GetBoolean("root/g2", false), true);
  EXPECT_EQ(frc::SmartDashboard::GetNumber("root/g3", 0.0), 8.46);
  EXPECT_EQ(frc::SmartDashboard::GetNumber("root/g4", 0.0), 846);
  EXPECT_EQ(frc::SmartDashboard::GetString("root/g5", ""), "846");
}