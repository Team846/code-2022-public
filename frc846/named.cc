#include "frc846/named.h"

#include <sstream>

namespace frc846 {

Named::Named(std::string_view name) : name_(name) {}

Named::Named(const Named& parent, std::string_view name)
    : name_(Named::Join({parent.name(), name})) {}

std::string Named::Join(std::initializer_list<std::string_view> elements) {
  std::stringstream ss;
  for (auto e : elements) {
    ss << e << "/";
  }

  std::string s = ss.str();
  return s.substr(0, s.length() - 1);  // get rid of trailing `/`
}

unsigned int Named::warn_count_ = 0;
unsigned int Named::error_count_ = 0;

}  // namespace frc846