/*
#include "frc846/pref.h"

#include <gtest/gtest.h>
#include <units/length.h>

class PrefTest : public testing::Test {
 protected:
  frc846::Named named{"root"};

  frc846::Pref<bool> p1{named, "p1", true};
  frc846::Pref<double> p2{named, "p2", 8.46};
  frc846::Pref<int> p3{named, "p3", 846};
  frc846::Pref<std::string> p4{named, "p4", "846"};
  frc846::Pref<units::inch_t> p5{named, "p5", 8.46_in};
};

// Test preferences are added to the table with the proper name.
TEST_F(PrefTest, TableContainsKey) {
  auto keys = {
      "root/p1", "root/p2", "root/p3", "root/p4", "root/p5 (in)",
  };

  for (const auto& k : keys) {
    EXPECT_TRUE(frc::Preferences::ContainsKey(k));
  }
}

// Test preferences are initialized with it's fallback values
TEST_F(PrefTest, InitializedFallbackValues) {
  EXPECT_EQ(p1.value(), true);
  EXPECT_EQ(p2.value(), 8.46);
  EXPECT_EQ(p3.value(), 846);
  EXPECT_EQ(p4.value(), "846");
  EXPECT_EQ(p5.value(), 8.46_in);
}

// TODO figure out how to test if values update when table updates.
*/