#pragma once

#include <stdexcept>
#include <string>

/**
 * @namespace communication_interfaces::exceptions
 * @brief Communication interfaces module for defining exception classes.
 */
namespace communication_interfaces::exceptions {

/*
 * @class SocketConfigurationException
 * @brief Exception thrown when a socket configuration fails
 */
class SocketConfigurationException : public std::runtime_error {
public:
  explicit SocketConfigurationException(const std::string& msg)
      : runtime_error("SocketConfigurationException: " + msg){};
};
}// namespace communication_interfaces::exceptions
