#include "clproto/encoders.hpp"

using namespace state_representation;

namespace clproto {

google::protobuf::RepeatedField<double> matrix_encoder(const Eigen::MatrixXd& matrix) {
  return encoder(std::vector<double>{matrix.data(), matrix.data() + matrix.size()});
}

proto::State encoder(const State& state) {
  proto::State message;
  message.set_name(state.get_name());
  message.set_empty(state.is_empty());
  return message;
}

proto::SpatialState encoder(const SpatialState& spatial_state) {
  proto::SpatialState message;
  *message.mutable_state() = encoder(static_cast<State>(spatial_state));
  message.set_reference_frame(spatial_state.get_reference_frame());
  return message;
}

proto::Vector3d encoder(const Eigen::Vector3d& vector) {
  proto::Vector3d message;
  message.set_x(vector.x());
  message.set_y(vector.y());
  message.set_z(vector.z());
  return message;
}

proto::Quaterniond encoder(const Eigen::Quaterniond& quaternion) {
  proto::Quaterniond message;
  message.set_w(quaternion.w());
  *message.mutable_vec() = encoder(Eigen::Vector3d(quaternion.vec()));
  return message;
}

proto::CartesianState encoder(const CartesianState& cartesian_state) {
  proto::CartesianState message;
  *message.mutable_spatial_state() = encoder(static_cast<SpatialState>(cartesian_state));
  if (cartesian_state.is_empty()) {
    return message;
  }
  *message.mutable_position() = encoder(cartesian_state.get_position());
  *message.mutable_orientation() = encoder(cartesian_state.get_orientation());
  *message.mutable_linear_velocity() = encoder(cartesian_state.get_linear_velocity());
  *message.mutable_angular_velocity() = encoder(cartesian_state.get_angular_velocity());
  *message.mutable_linear_acceleration() = encoder(cartesian_state.get_linear_acceleration());
  *message.mutable_angular_acceleration() = encoder(cartesian_state.get_angular_acceleration());
  *message.mutable_force() = encoder(cartesian_state.get_force());
  *message.mutable_torque() = encoder(cartesian_state.get_torque());
  return message;
}

proto::Jacobian encoder(const Jacobian& jacobian) {
  proto::Jacobian message;
  *message.mutable_state() = encoder(static_cast<State>(jacobian));
  *message.mutable_joint_names() = {jacobian.get_joint_names().begin(), jacobian.get_joint_names().end()};
  message.set_frame(jacobian.get_frame());
  message.set_reference_frame(jacobian.get_reference_frame());
  message.set_rows(jacobian.rows());
  message.set_cols(jacobian.cols());
  if (jacobian.is_empty()) {
    return message;
  }
  *message.mutable_data() = matrix_encoder(jacobian.data());
  return message;
}

proto::JointState encoder(const JointState& joint_state) {
  proto::JointState message;
  *message.mutable_state() = encoder(static_cast<State>(joint_state));
  *message.mutable_joint_names() = {joint_state.get_names().begin(), joint_state.get_names().end()};
  if (joint_state.is_empty()) {
    return message;
  }
  *message.mutable_positions() = matrix_encoder(joint_state.get_positions());
  *message.mutable_velocities() = matrix_encoder(joint_state.get_velocities());
  *message.mutable_accelerations() = matrix_encoder(joint_state.get_accelerations());
  *message.mutable_torques() = matrix_encoder(joint_state.get_torques());
  return message;
}

template<>
proto::Parameter encoder(proto::Parameter& message, const Parameter<int>& parameter) {
  if (parameter) {
    message.mutable_parameter_value()->mutable_int_()->set_value(parameter.get_value());
  } else {
    message.mutable_parameter_value()->mutable_int_()->set_value(int());
  }
  return message;
}

template<>
proto::Parameter encoder(proto::Parameter& message, const Parameter<std::vector<int>>& parameter) {
  if (parameter) {
    *message.mutable_parameter_value()->mutable_int_array()->mutable_value() =
        {parameter.get_value().begin(), parameter.get_value().end()};
  } else {
    *message.mutable_parameter_value()->mutable_int_array()->mutable_value() = {};
  }
  return message;
}

template<>
proto::Parameter encoder(proto::Parameter& message, const Parameter<double>& parameter) {
  if (parameter) {
    message.mutable_parameter_value()->mutable_double_()->set_value(parameter.get_value());
  } else {
    message.mutable_parameter_value()->mutable_double_()->set_value(double());
  }
  return message;
}

template<>
proto::Parameter encoder(proto::Parameter& message, const Parameter<std::vector<double>>& parameter) {
  if (parameter) {
    *message.mutable_parameter_value()->mutable_double_array()->mutable_value() =
        {parameter.get_value().begin(), parameter.get_value().end()};
  } else {
    *message.mutable_parameter_value()->mutable_double_array()->mutable_value() = {};
  }
  return message;
}

template<>
proto::Parameter encoder(proto::Parameter& message, const Parameter<bool>& parameter) {
  if (parameter) {
    message.mutable_parameter_value()->mutable_bool_()->set_value(parameter.get_value());
  } else {
    message.mutable_parameter_value()->mutable_bool_()->set_value(bool());
  }
  return message;
}

template<>
proto::Parameter encoder(proto::Parameter& message, const Parameter<std::vector<bool>>& parameter) {
  if (parameter) {
    *message.mutable_parameter_value()->mutable_bool_array()->mutable_value() =
        {parameter.get_value().begin(), parameter.get_value().end()};
  } else {
    *message.mutable_parameter_value()->mutable_bool_array()->mutable_value() = {};
  }
  return message;
}

template<>
proto::Parameter encoder(proto::Parameter& message, const Parameter<std::string>& parameter) {
  if (parameter) {
    message.mutable_parameter_value()->mutable_string()->set_value(parameter.get_value());
  } else {
    message.mutable_parameter_value()->mutable_string()->set_value(std::string ());
  }
  return message;
}

template<>
proto::Parameter
encoder(proto::Parameter& message, const Parameter<std::vector<std::string>>& parameter) {
  if (parameter) {
    *message.mutable_parameter_value()->mutable_string_array()->mutable_value() =
        {parameter.get_value().begin(), parameter.get_value().end()};
  } else {
    *message.mutable_parameter_value()->mutable_string_array()->mutable_value() = {};
  }
  return message;
}

template<>
proto::Parameter encoder(proto::Parameter& message, const Parameter<Eigen::VectorXd>& parameter) {
  if (parameter) {
    *message.mutable_parameter_value()->mutable_vector()->mutable_value() = matrix_encoder(parameter.get_value());
  } else {
    *message.mutable_parameter_value()->mutable_vector()->mutable_value() = matrix_encoder(Eigen::VectorXd());
  }
  return message;
}

template<>
proto::Parameter encoder(proto::Parameter& message, const Parameter<Eigen::MatrixXd>& parameter) {
  if (parameter) {
    *message.mutable_parameter_value()->mutable_matrix()->mutable_value() = matrix_encoder(parameter.get_value());
    message.mutable_parameter_value()->mutable_matrix()->set_rows(parameter.get_value().rows());
    message.mutable_parameter_value()->mutable_matrix()->set_cols(parameter.get_value().cols());
  } else {
    *message.mutable_parameter_value()->mutable_matrix()->mutable_value() = matrix_encoder(Eigen::MatrixXd());
    message.mutable_parameter_value()->mutable_matrix()->set_rows(0);
    message.mutable_parameter_value()->mutable_matrix()->set_cols(0);
  }
  return message;
}

proto::AnalogIOState encoder(const AnalogIOState& io_state) {
  proto::AnalogIOState message;
  *message.mutable_state() = encoder(static_cast<State>(io_state));
  *message.mutable_io_names() = {io_state.get_names().begin(), io_state.get_names().end()};
  if (io_state.is_empty()) {
    return message;
  }
  std::vector<bool> vec;
  vec.resize(io_state.get_size());
  for (unsigned int i = 0; i < vec.size(); ++i) {
    vec.at(i) = io_state.data()(i);
  }
  *message.mutable_values() = encoder(vec);
  return message;
}

proto::DigitalIOState encoder(const DigitalIOState& io_state) {
  proto::DigitalIOState message;
  *message.mutable_state() = encoder(static_cast<State>(io_state));
  *message.mutable_io_names() = {io_state.get_names().begin(), io_state.get_names().end()};
  if (io_state.is_empty()) {
    return message;
  }
  *message.mutable_values() = matrix_encoder(io_state.data());
  return message;
}
}