#pragma once

#include <stdexcept>
#include <string>

namespace state_representation::exceptions {
class NoSolutionToFitException : public std::runtime_error {
public:
  explicit NoSolutionToFitException(const std::string& msg) : runtime_error(msg){};
};
}// namespace state_representation::exceptions
