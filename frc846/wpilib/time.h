#ifndef FRC846_TIME_H_
#define FRC846_TIME_H_

#include <hal/HALBase.h>
#include <units/time.h>

namespace frc846 {
namespace wpilib {

// Get the current time.
units::second_t CurrentFPGATime();

}  // namespace wpilib
}  // namespace frc846

#endif  // FRC846_TIME_H_