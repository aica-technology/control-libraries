#pragma once

#include <stdexcept>
#include <string>

/**
 * @namespace communication_interfaces::exceptions
 * @brief Communication interfaces module for defining exception classes.
 */
namespace communication_interfaces::exceptions {

/**
 * @class SocketException
 * @brief Base class for communication interfaces exceptions.
 * @details This inherits from std::runtime_error.
 */
class SocketException : public std::runtime_error {
public:
  explicit SocketException(const std::string& msg) : SocketException("SocketException", msg) {}

protected:
  SocketException(const std::string& prefix, const std::string& msg) : std::runtime_error(prefix + ": " + msg) {}
};

/*
 * @class SocketConfigurationException
 * @brief Exception thrown when a socket configuration fails
 */
class SocketConfigurationException : public SocketException {
public:
  explicit SocketConfigurationException(const std::string& msg)
      : SocketException("SocketConfigurationException", msg) {}
};
}// namespace communication_interfaces::exceptions
