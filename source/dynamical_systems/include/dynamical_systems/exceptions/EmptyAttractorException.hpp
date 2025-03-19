#pragma once

#include <stdexcept>
#include <string>

namespace dynamical_systems::exceptions {
class EmptyAttractorException : public std::runtime_error {
public:
  explicit EmptyAttractorException(const std::string& msg) : runtime_error(msg){};
};
}// namespace dynamical_systems::exceptions
