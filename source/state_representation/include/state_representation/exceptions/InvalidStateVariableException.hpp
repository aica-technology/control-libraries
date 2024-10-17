#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {
class InvalidVariableStateException : public std::runtime_error {
public:
  explicit InvalidVariableStateException(const std::string& msg) : runtime_error(msg) {};
};
}// namespace state_representation::exceptions
