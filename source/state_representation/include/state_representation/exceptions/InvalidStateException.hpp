#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {
class InvalidStateException : public std::runtime_error {
public:
  explicit InvalidStateException(const std::string& msg) : runtime_error(msg) {};
};
}// namespace state_representation::exceptions
