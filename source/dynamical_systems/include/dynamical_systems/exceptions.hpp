#pragma once

#include "state_representation/exceptions.hpp"

/**
 * @namespace dynamical_systems::exceptions
 * @brief Dynamical systems module for defining exception classes.
 */
namespace dynamical_systems::exceptions {

/**
 * @class EmptyAttractorException
 * @brief Exception thrown when an attractor is empty.
 */
class EmptyAttractorException : public state_representation::exceptions::Exception {
public:
  explicit EmptyAttractorException(const std::string& msg) : Exception("EmptyAttractorException", msg){};
};

/**
 * @class EmptyBaseFrameException
 * @brief Exception thrown when a base frame is empty.
 */
class EmptyBaseFrameException : public state_representation::exceptions::Exception {
public:
  explicit EmptyBaseFrameException(const std::string& msg) : Exception("EmptyBaseFrameException", msg){};
};

/**
 * @class InvalidDynamicalSystemException
 * @brief Exception thrown when a dynamical system is invalid.
 */
class InvalidDynamicalSystemException : public state_representation::exceptions::Exception {
public:
  explicit InvalidDynamicalSystemException(const std::string& msg) : Exception("InvalidDynamicalSystemException", msg){};
};
}// namespace dynamical_systems::exceptions
