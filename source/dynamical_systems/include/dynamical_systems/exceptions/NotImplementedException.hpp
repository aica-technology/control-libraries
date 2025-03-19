#pragma once

#include <stdexcept>
#include <string>

namespace dynamical_systems::exceptions {
class NotImplementedException : public std::logic_error {
public:
  explicit NotImplementedException(const std::string& msg) : logic_error(msg){};
};
}// namespace dynamical_systems::exceptions
