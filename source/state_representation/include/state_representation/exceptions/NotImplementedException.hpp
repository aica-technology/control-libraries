#pragma once

#include <stdexcept>
#include <string>

namespace state_representation::exceptions {
class NotImplementedException : public std::logic_error {
public:
  explicit NotImplementedException(const std::string& msg) : logic_error(msg){};
};
}// namespace state_representation::exceptions
