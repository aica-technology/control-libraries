#include "clproto/decoders.h"

using namespace state_representation;

namespace clproto {

std::vector<bool> decoder(const google::protobuf::RepeatedField<bool>& message) {
  // explicit construction is needed for the bool vector due to stl optimisations
  std::vector<bool> vec(message.begin(), message.end());
  return vec;
}

Eigen::Vector3d decoder(const proto::Vector3d& message) {
  return {message.x(), message.y(), message.z()};
}

Eigen::Quaterniond decoder(const proto::Quaterniond& message) {
  return {message.w(), message.vec().x(), message.vec().y(), message.vec().z()};
}

template<>
Parameter<int> decoder(const state_representation::proto::Parameter& message) {
  if (message.state().empty()) {
    return Parameter<int>(message.state().name());
  }
  return Parameter<int>(message.state().name(), message.parameter_value().int_().value());
}

template<>
Parameter<std::vector<int>> decoder(const state_representation::proto::Parameter& message) {
  if (message.state().empty()) {
    return Parameter<std::vector<int>>(message.state().name());
  }
  return Parameter<std::vector<int>>(
      message.state().name(), decoder(message.parameter_value().int_array().value()));
}

template<>
Parameter<double> decoder(const state_representation::proto::Parameter& message) {
  if (message.state().empty()) {
    return Parameter<double>(message.state().name());
  }
  return Parameter<double>(message.state().name(), message.parameter_value().double_().value());
}

template<>
Parameter<std::vector<double>> decoder(const state_representation::proto::Parameter& message) {
  if (message.state().empty()) {
    return Parameter<std::vector<double>>(message.state().name());
  }
  return Parameter<std::vector<double>>(
      message.state().name(), decoder(message.parameter_value().double_array().value()));
}

template<>
Parameter<bool> decoder(const state_representation::proto::Parameter& message) {
  if (message.state().empty()) {
    return Parameter<bool>(message.state().name());
  }
  return Parameter<bool>(message.state().name(), message.parameter_value().bool_().value());
}

template<>
Parameter<std::vector<bool>> decoder(const state_representation::proto::Parameter& message) {
  if (message.state().empty()) {
    return Parameter<std::vector<bool>>(message.state().name());
  }
  auto val = decoder(message.parameter_value().bool_array().value());
  return Parameter<std::vector<bool>>(message.state().name(), val);
}

template<>
Parameter<std::string> decoder(const state_representation::proto::Parameter& message) {
  if (message.state().empty()) {
    return Parameter<std::string>(message.state().name());
  }
  return Parameter<std::string>(message.state().name(), message.parameter_value().string().value());
}

template<>
Parameter<std::vector<std::string>> decoder(const state_representation::proto::Parameter& message) {
  if (message.state().empty()) {
    return Parameter<std::vector<std::string>>(message.state().name());
  }
  return Parameter<std::vector<std::string>>(
      message.state().name(), decoder(message.parameter_value().string_array().value()));
}

template<>
Parameter<Eigen::VectorXd> decoder(const state_representation::proto::Parameter& message) {
  if (message.state().empty()) {
    return Parameter<Eigen::VectorXd>(message.state().name());
  }
  std::vector<double> elements = decoder(message.parameter_value().vector().value());
  return Parameter<Eigen::VectorXd>(
      message.state().name(), Eigen::Map<Eigen::VectorXd>(
          elements.data(), static_cast<Eigen::Index>(elements.size())));
}

template<>
Parameter<Eigen::MatrixXd> decoder(const state_representation::proto::Parameter& message) {
  if (message.state().empty()) {
    return Parameter<Eigen::MatrixXd>(message.state().name());
  }
  std::vector<double> elements = decoder(message.parameter_value().matrix().value());
  return Parameter<Eigen::MatrixXd>(
      message.state().name(), Eigen::Map<Eigen::MatrixXd>(
          elements.data(), message.parameter_value().matrix().rows(), message.parameter_value().matrix().cols()));
}
}