#include "frc846/sendable_callback.h"

#include <wpi/sendable/SendableBuilder.h>

namespace frc846 {

SendableCallback::SendableCallback(std::function<void()> callback)
    : callback_(callback) {}

void SendableCallback::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Command");
  builder.AddStringProperty(
      ".name", [] { return "Run"; }, nullptr);
  builder.AddBooleanProperty(
      "running", [] { return false; },
      [this](bool value) {
        if (value) {
          callback_();
        }
      });
}

}  // namespace frc846