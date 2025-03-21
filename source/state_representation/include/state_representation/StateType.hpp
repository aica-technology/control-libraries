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
  NONE,
  STATE,
  SPATIAL_STATE,
  CARTESIAN_STATE,
  CARTESIAN_POSE,
  CARTESIAN_TWIST,
  CARTESIAN_ACCELERATION,
  CARTESIAN_WRENCH,
  JOINT_STATE,
  JOINT_POSITIONS,
  JOINT_VELOCITIES,
  JOINT_ACCELERATIONS,
  JOINT_TORQUES,
  JACOBIAN,
  PARAMETER,
  GEOMETRY_SHAPE,
  GEOMETRY_ELLIPSOID,
  TRAJECTORY,
  DIGITAL_IO_STATE,
  ANALOG_IO_STATE,
#ifdef EXPERIMENTAL_FEATURES
  DUAL_QUATERNION_STATE,
  DUAL_QUATERNION_POSE,
  DUAL_QUATERNION_TWIST
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
    case StateType::TRAJECTORY:
      return "Trajectory";
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
