#pragma once

#include <exception>
#include <iostream>

namespace communication_interfaces::exceptions {

/*
 * @class SocketConfigurationException
 * @brief Exception that is thrown when a socket configuration failed
 */
class SocketConfigurationException : public std::runtime_error {
public:
  explicit SocketConfigurationException(const std::string& msg) : runtime_error(msg) {};
};
} // namespace communication_interfaces::exceptions
