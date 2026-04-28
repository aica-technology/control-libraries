#pragma once

#include "state_representation/exceptions.hpp"

/**
 * @namespace controllers::exceptions
 * @brief Controllers module for defining exception classes.
 */
namespace controllers::exceptions {

/**
 * @class InvalidControllerException
 * @brief Exception thrown when an invalid controller is encountered.
 */
class InvalidControllerException : public state_representation::exceptions::Exception {
public:
  explicit InvalidControllerException(const std::string& msg) : Exception("InvalidControllerException", msg){};
};

/**
 * @class NoRobotModelException
 * @brief Exception thrown when no robot model is available.
 */
class NoRobotModelException : public state_representation::exceptions::Exception {
public:
  explicit NoRobotModelException(const std::string& msg) : Exception("NoRobotModelException", msg){};
};
}// namespace controllers::exceptions
