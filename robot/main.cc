#include <frc/RobotBase.h>

#include "robot/funky_robot.h"

int main() {
  std::cout << "Starting robot code..." << std::endl;
  return frc::StartRobot<FunkyRobot>();
}