#pragma once

#include <stdexcept>
#include <string>

/**
 * @namespace state_representation::exceptions
 * @brief State representation module for defining exception classes.
 */
namespace state_representation::exceptions {

/**
 * @class Exception
 * @brief Base class for state representation exceptions.
 * @details This inherits from std::runtime_error.
 */
class Exception : public std::runtime_error {
public:
  explicit Exception(const std::string& msg) : Exception("Exception", msg) {}

protected:
  Exception(const std::string& prefix, const std::string& msg) : std::runtime_error(prefix + ": " + msg) {}
};

/**
 * @class EmptyStateException
 * @brief Exception thrown when accessing data of an empty state.
 */
class EmptyStateException : public Exception {
public:
  explicit EmptyStateException(const std::string& msg) : Exception("EmptyStateException", msg) {}
};

/**
 * @class IncompatibleReferenceFramesException
 * @brief Exception thrown when states have incompatible reference frames.
 */
class IncompatibleReferenceFramesException : public Exception {
public:
  explicit IncompatibleReferenceFramesException(const std::string& msg)
      : Exception("IncompatibleReferenceFramesException", msg) {}
};

/**
 * @class IncompatibleSizeException
 * @brief Exception thrown when states have incompatible sizes.
 */
class IncompatibleSizeException : public Exception {
public:
  explicit IncompatibleSizeException(const std::string& msg) : Exception("IncompatibleSizeException", msg) {}
};

/**
 * @class IncompatibleStatesException
 * @brief Exception thrown when states are incompatible.
 */
class IncompatibleStatesException : public Exception {
public:
  explicit IncompatibleStatesException(const std::string& msg) : Exception("IncompatibleStatesException", msg) {}
};

/**
 * @class InvalidCastException
 * @brief Exception thrown when an invalid cast is attempted.
 */
class InvalidCastException : public Exception {
public:
  explicit InvalidCastException(const std::string& msg) : Exception("InvalidCastException", msg) {}
};

/**
 * @class InvalidParameterException
 * @brief Exception thrown when an invalid parameter is provided.
 */
class InvalidParameterException : public Exception {
public:
  explicit InvalidParameterException(const std::string& msg) : Exception("InvalidParameterException", msg) {}
};

/**
 * @class InvalidPointerException
 * @brief Exception thrown when an invalid pointer is encountered.
 */
class InvalidPointerException : public Exception {
public:
  explicit InvalidPointerException(const std::string& msg) : Exception("InvalidPointerException", msg) {}
};

/**
 * @class InvalidStateVariableException
 * @brief Exception thrown when an invalid state variable is provided.
 */
class InvalidStateVariableException : public Exception {
public:
  explicit InvalidStateVariableException(const std::string& msg) : Exception("InvalidStateVariableException", msg) {}
};

/**
 * @class IONotFoundException
 * @brief Exception thrown when a IO name or index is out of range
 */
class IONotFoundException : public Exception {
public:
  explicit IONotFoundException(const std::string& msg) : Exception("IONotFoundException", msg) {}
};

/**
 * @class JointNotFoundException
 * @brief Exception thrown when a joint name or index is out of range
 */
class JointNotFoundException : public Exception {
public:
  explicit JointNotFoundException(const std::string& msg) : Exception("JointNotFoundException", msg) {}
};

/**
 * @class NotImplementedException
 * @brief Exception thrown when a method is not implemented.
 */
class NotImplementedException : public Exception {
public:
  explicit NotImplementedException(const std::string& msg) : Exception("NotImplementedException", msg) {}
};
}// namespace state_representation::exceptions
