#pragma once

#include <stdexcept>
#include <string>

namespace dynamical_systems::exceptions {
class IncompatibleSizeException : public std::runtime_error {
public:
  explicit IncompatibleSizeException(const std::string& msg) : runtime_error(msg){};
};
}// namespace dynamical_systems::exceptions
