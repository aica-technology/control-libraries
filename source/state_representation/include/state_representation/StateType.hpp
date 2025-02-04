#pragma once

#include <string>

/**
 * @namespace state_representation
 * @brief Core state variables and objects
 */
namespace state_representation {

/**
 * @enum StateType
 * @brief The class types inheriting from State
 */
enum class StateType {
  NONE = 0,
  STATE = 1,
  SPATIAL_STATE = 2,
  CARTESIAN_STATE = 3,
  CARTESIAN_POSE = 4,
  CARTESIAN_TWIST = 5,
  CARTESIAN_ACCELERATION = 6,
  CARTESIAN_WRENCH = 7,
  JOINT_STATE = 8,
  JOINT_POSITIONS = 9,
  JOINT_VELOCITIES = 10,
  JOINT_ACCELERATIONS = 11,
  JOINT_TORQUES = 12,
  JACOBIAN = 13,
  PARAMETER = 14,
  GEOMETRY_SHAPE = 15,
  GEOMETRY_ELLIPSOID = 16,
  DIGITAL_IO_STATE = 18,
  ANALOG_IO_STATE = 19,
  CARTESIAN_TRAJECTORY = 20,
  JOINT_TRAJECTORY = 21,
#ifdef EXPERIMENTAL_FEATURES
  DUAL_QUATERNION_STATE = 22,
  DUAL_QUATERNION_POSE = 23,
  DUAL_QUATERNION_TWIST = 24
#endif
};

/**
 * @brief Convert state type enum to its corresponding name
 */
[[maybe_unused]] static std::string get_state_type_name(const StateType& state_type) {
  switch (state_type) {
    case StateType::STATE:
      return "State";
    case StateType::SPATIAL_STATE:
      return "SpatialState";
    case StateType::CARTESIAN_STATE:
      return "CartesianState";
    case StateType::CARTESIAN_POSE:
      return "CartesianPose";
    case StateType::CARTESIAN_TWIST:
      return "CartesianTwist";
    case StateType::CARTESIAN_ACCELERATION:
      return "CartesianAcceleration";
    case StateType::CARTESIAN_WRENCH:
      return "CartesianWrench";
    case StateType::JOINT_STATE:
      return "JointState";
    case StateType::JOINT_POSITIONS:
      return "JointPositions";
    case StateType::JOINT_VELOCITIES:
      return "JointVelocities";
    case StateType::JOINT_ACCELERATIONS:
      return "JointAccelerations";
    case StateType::JOINT_TORQUES:
      return "JointTorques";
    case StateType::JACOBIAN:
      return "Jacobian";
    case StateType::PARAMETER:
      return "Parameter";
    case StateType::GEOMETRY_SHAPE:
      return "Shape";
    case StateType::GEOMETRY_ELLIPSOID:
      return "Ellipsoid";
    case StateType::CARTESIAN_TRAJECTORY:
      return "CartesianTrajectory";
    case StateType::JOINT_TRAJECTORY:
      return "JointTrajectory";
    case StateType::DIGITAL_IO_STATE:
      return "DigitalIOState";
    case StateType::ANALOG_IO_STATE:
      return "AnalogIOState";
#ifdef EXPERIMENTAL_FEATURES
    case StateType::DUAL_QUATERNION_STATE:
      return "DualQuaternionState";
    case StateType::DUAL_QUATERNION_POSE:
      return "DualQuaternionPose";
    case StateType::DUAL_QUATERNION_TWIST:
      return "DualQuaternionTwist";
#endif
    default:
      return "";
  }
}

}// namespace state_representation
