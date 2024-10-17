#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {
class InvalidStateVariableException : public std::runtime_error {
public:
  explicit InvalidStateVariableException(const std::string& msg) : runtime_error(msg) {};
};
}// namespace state_representation::exceptions
