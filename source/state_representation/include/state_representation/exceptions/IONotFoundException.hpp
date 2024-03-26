#pragma once

#include <exception>
#include <iostream>

namespace state_representation::exceptions {

/**
 * @class IONotFoundException
 * @brief Exception that is thrown when a IO name or index is out of range
 */
class IONotFoundException : public std::logic_error {
public:
  explicit IONotFoundException(const std::string& msg) : logic_error(msg) {};
};
}// namespace state_representation::exceptions
