#ifndef ROBOT_FIELD_H_
#define ROBOT_FIELD_H_

#include "frc846/math.h"

struct field {
  struct hub {
    // Height from ground to top of the upper hub.
    //
    // Game manual page 23.
    static constexpr units::foot_t kTopHeight = 8_ft + 8_in;

    // Diameter of upper hub INCLUDING vision tape ring.
    //
    // RAPID REACT season specific field drawings page 78.
    static constexpr units::foot_t kOuterDiameter = 53.38_in;
  };

  // Various field points for autonomous routines.
  //
  // TODO wait until https://github.com/wpilibsuite/allwpilib/issues/4137 is
  // fixed.
  //
  // can't have negative signs in unit fields lol
  struct points {
    static frc846::Position kRightStart() {
      return {{7.50_ft, 0.40_ft}, 88.5_deg};
    }

    static frc846::Position kRightIntake1() {
      return {{11.35_ft, -2.16_ft - 1_ft}, 90.0_deg};
    };

    static frc846::Position kRightIntake2() {
      return {{7.89_ft, -9.32_ft}, -154.2_deg};
    }

    static frc846::Position kRightShoot1() {
      return {{7.72_ft, -7.25_ft}, 133.2_deg};
    }

    static frc846::Position kLeftStart() {
      return {{-5.32_ft, -5.31_ft}, -136.5_deg};
    }

    static frc846::Position kLeftIntake1() {
      return {{-6.49_ft, -9.61_ft}, -164.9_deg};
    }

    static frc846::Position kLeftSteal1() {
      return {{-9.53_ft, -8.20_ft}, -46.5_deg};
    }

    static frc846::Position kLeftSteal2() {
      return {{1.66_ft, -12.04_ft}, 109.0_deg};
    }

    static frc846::Position kLeftDump() {
      return {{1.66_ft, -22.00_ft}, 90.0_deg};
    }

    static frc846::Position kTerminalIntake() {
      return {{8.97_ft, -22.63_ft + 0.8_ft}, 136.3_deg};
    }
  };
};

#endif  // ROBOT_FIELD_H_