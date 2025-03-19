#pragma once

#include <stdexcept>
#include <string>

namespace state_representation::exceptions {
class IncompatibleStatesException : public std::logic_error {
public:
  explicit IncompatibleStatesException(const std::string& msg) : logic_error(msg){};
};
}// namespace state_representation::exceptions
