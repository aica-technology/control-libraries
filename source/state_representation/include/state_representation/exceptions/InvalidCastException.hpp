#pragma once

#include <stdexcept>
#include <string>

namespace state_representation::exceptions {
class InvalidCastException : public std::runtime_error {
  public:
    explicit InvalidCastException(const std::string& msg) : runtime_error(msg) {};
};
}