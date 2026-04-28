#pragma once

#include "state_representation/exceptions.hpp"

/**
 * @namespace robot_model::exceptions
 * @brief Robot model module for defining exception classes.
 */
namespace robot_model::exceptions {

/**
 * @class CollisionGeometryException
 * @brief Exception thrown when there is an error related to collision geometry.
 */
class CollisionGeometryException : public state_representation::exceptions::Exception {
public:
  explicit CollisionGeometryException(const std::string& error_message)
      : Exception("CollisionGeometryException", error_message) {}
};

/**
 * @class FrameNotFoundException
 * @brief Exception thrown when a frame with a specified name or ID is not found in the robot model.
 */
class FrameNotFoundException : public state_representation::exceptions::Exception {
public:
  explicit FrameNotFoundException(const std::string& frame)
      : Exception("FrameNotFoundException", "Frame with name or ID " + frame + " is not in the robot model") {}
};

/**
 * @class InvalidJointStateSizeException
 * @brief Exception thrown when the size of the joint state does not match the expected number of joints in the robot
 *model.
 */
class InvalidJointStateSizeException : public state_representation::exceptions::Exception {
public:
  explicit InvalidJointStateSizeException(unsigned int state_nb_joints, unsigned int robot_nb_joints)
      : Exception(
            "InvalidJointStateSizeException",
            "The robot has " + std::to_string(robot_nb_joints) + " joints, but the current joint state size "
                + std::to_string(state_nb_joints) + "."
        ) {}
};

class InverseKinematicsNotConvergingException : public state_representation::exceptions::Exception {
public:
  InverseKinematicsNotConvergingException(unsigned int iterations, double error)
      : Exception(
            "InverseKinematicsNotConvergingException",
            "The inverse kinematics algorithm did not converge.\nThe residual error after " + std::to_string(iterations)
                + " iterations is " + std::to_string(error) + "."
        ) {}
};
}// namespace robot_model::exceptions
