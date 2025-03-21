#pragma once

#include <stdexcept>
#include <string>

namespace state_representation::exceptions {
class InvalidParameterException : public std::runtime_error {
public:
  explicit InvalidParameterException(const std::string& msg) : runtime_error(msg){};
};
}// namespace state_representation::exceptions
