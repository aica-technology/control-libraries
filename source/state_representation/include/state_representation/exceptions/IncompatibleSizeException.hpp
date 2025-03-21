#pragma once

#include <stdexcept>
#include <string>

namespace state_representation::exceptions {
class IncompatibleSizeException : public std::logic_error {
public:
  explicit IncompatibleSizeException(const std::string& msg) : logic_error(msg){};
};
}// namespace state_representation::exceptions
