#pragma once

#include <stdexcept>
#include <string>

namespace dynamical_systems::exceptions {
class EmptyBaseFrameException : public std::runtime_error {
public:
  explicit EmptyBaseFrameException(const std::string& msg) : runtime_error(msg){};
};
}// namespace dynamical_systems::exceptions
