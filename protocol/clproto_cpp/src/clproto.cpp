#include "clproto.hpp"

#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/type_resolver_util.h>

#include "clproto/decoders.hpp"
#include "clproto/encoders.hpp"

#include <state_representation/AnalogIOState.hpp>
#include <state_representation/DigitalIOState.hpp>
#include <state_representation/State.hpp>
#include <state_representation/parameters/Parameter.hpp>
#include <state_representation/space/Jacobian.hpp>
#include <state_representation/space/SpatialState.hpp>
#include <state_representation/space/cartesian/CartesianAcceleration.hpp>
#include <state_representation/space/cartesian/CartesianPose.hpp>
#include <state_representation/space/cartesian/CartesianState.hpp>
#include <state_representation/space/cartesian/CartesianTwist.hpp>
#include <state_representation/space/cartesian/CartesianWrench.hpp>
#include <state_representation/space/joint/JointAccelerations.hpp>
#include <state_representation/space/joint/JointPositions.hpp>
#include <state_representation/space/joint/JointState.hpp>
#include <state_representation/space/joint/JointTorques.hpp>
#include <state_representation/space/joint/JointVelocities.hpp>

#include "state_representation/state_message.pb.h"

using namespace state_representation;

namespace clproto {

DecodingException::DecodingException(const std::string& msg) : std::runtime_error(msg) {}

JsonParsingException::JsonParsingException(const std::string& msg) : std::runtime_error(msg) {}

bool is_valid(const std::string& msg) {
  return check_message_type(msg) != MessageType::UNKNOWN_MESSAGE;
}

MessageType check_message_type(const std::string& msg) {
  if (proto::StateMessage message; message.ParseFromString(msg)) {
    return static_cast<MessageType>(message.message_type_case());
  }

  /* In theory, sending "raw" messages (message types without the
   * StateMessage wrapper) could also be supported, though it would
   * require manually checking each case as in the following snippet:

  if (proto::State message; message.ParseFromString(msg)) {
    return MessageType::STATE_MESSAGE;
  } else if (proto::SpatialState message; message.ParseFromString(msg)) {
    return MessageType::SPATIAL_STATE_MESSAGE;
  }

   * Because the intention is to encode / decode messages using
   * this library, and all encoded messages use the StateMessage
   * wrapper, raw messages are treated as UNKNOWN at this time.
   */

  return MessageType::UNKNOWN_MESSAGE;
}

ParameterMessageType check_parameter_message_type(const std::string& msg) {
  if (proto::StateMessage message; message.ParseFromString(msg) && message.has_parameter()) {
    return static_cast<ParameterMessageType>(message.parameter().parameter_value().value_type_case());
  }
  return ParameterMessageType::UNKNOWN_PARAMETER;
}

// --- Serialization methods --- //

void pack_fields(const std::vector<std::string>& fields, char* data) {
  std::size_t index = 0;
  field_length_t nfields;
  field_length_t sizes[CLPROTO_PACKING_MAX_FIELDS];

  // write out the number of fields
  nfields = static_cast<field_length_t>(fields.size());
  memcpy(data, &nfields, sizeof(field_length_t));
  index += sizeof(field_length_t);

  // write out the data size of each field
  for (std::size_t field = 0; field < nfields; ++field) {
    sizes[field] = static_cast<field_length_t>(fields.at(field).size());
  }
  memcpy(&data[index], sizes, nfields * sizeof(field_length_t));
  index += nfields * sizeof(field_length_t);

  // write out each field
  for (std::size_t field = 0; field < nfields; ++field) {
    memcpy(&data[index], fields.at(field).c_str(), sizes[field]);
    index += sizes[field];
  }
}

std::vector<std::string> unpack_fields(const char* data) {
  std::size_t index = 0;
  field_length_t nfields;
  field_length_t sizes[CLPROTO_PACKING_MAX_FIELDS];
  char field_buffer[CLPROTO_PACKING_MAX_FIELD_LENGTH];
  std::vector<std::string> fields;

  // read out the number of fields
  memcpy(&nfields, data, sizeof(field_length_t));
  index += sizeof(field_length_t);

  // read out the data size of each field
  memcpy(sizes, &data[index], nfields * sizeof(field_length_t));
  index += nfields * sizeof(field_length_t);

  // read out each field
  for (std::size_t field = 0; field < nfields; ++field) {
    memcpy(field_buffer, &data[index], sizes[field]);
    fields.emplace_back(std::string(field_buffer, sizes[field]));
    index += sizes[field];
  }
  return fields;
}

// --- JSON utilities --- //

std::string to_json(const std::string& msg) {
  std::string json;

  auto resolver = std::unique_ptr<google::protobuf::util::TypeResolver>{
      google::protobuf::util::NewTypeResolverForDescriptorPool("", google::protobuf::DescriptorPool::generated_pool())
  };

  auto status = google::protobuf::util::BinaryToJsonString(
      resolver.get(), "/state_representation.proto.StateMessage", msg, std::addressof(json)
  );

  if (!status.ok() || json.size() <= 2) {
    throw JsonParsingException("Could not parse the binary data into a JSON formatted state message");
  }
  return json;
}

std::string from_json(const std::string& json) {
  std::string msg;

  auto resolver = std::unique_ptr<google::protobuf::util::TypeResolver>{
      google::protobuf::util::NewTypeResolverForDescriptorPool("", google::protobuf::DescriptorPool::generated_pool())
  };

  auto status = google::protobuf::util::JsonToBinaryString(
      resolver.get(), "/state_representation.proto.StateMessage", json, std::addressof(msg)
  );

  if (!status.ok()) {
    throw JsonParsingException("Could not parse a valid state from the JSON message: " + json);
  }

  return msg;
}

/* ----------------------
 *         State
 * ---------------------- */
template<>
std::string encode<State>(const State& obj);
template<>
State decode(const std::string& msg);
template<>
bool decode(const std::string& msg, State& obj);
template<>
std::string encode<State>(const State& obj) {
  proto::StateMessage message;
  *message.mutable_state() = encoder(obj);
  return message.SerializeAsString();
}
template<>
State decode(const std::string& msg) {
  State obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a State");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, State& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg) && message.message_type_case() == proto::StateMessage::MessageTypeCase::kState
        )) {
      return false;
    }

    auto state = message.state();
    obj = State(state.name());

    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *     AnalogIOState
 * ---------------------- */
template<>
std::string encode<AnalogIOState>(const AnalogIOState& obj);
template<>
AnalogIOState decode(const std::string& msg);
template<>
bool decode(const std::string& msg, AnalogIOState& obj);
template<>
std::string encode<AnalogIOState>(const AnalogIOState& obj) {
  proto::StateMessage message;
  *message.mutable_analog_io_state() = encoder(obj);
  return message.SerializeAsString();
}
template<>
AnalogIOState decode(const std::string& msg) {
  AnalogIOState obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a AnalogIOState");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, AnalogIOState& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kAnalogIoState)) {
      return false;
    }

    auto state = message.analog_io_state();
    obj = AnalogIOState(state.state().name(), decoder(state.io_names()));
    if (!state.state().empty()) {
      obj.set_data(decoder(state.values()));
    };
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *     DigitalIOState
 * ---------------------- */
template<>
std::string encode<DigitalIOState>(const DigitalIOState& obj);
template<>
DigitalIOState decode(const std::string& msg);
template<>
bool decode(const std::string& msg, DigitalIOState& obj);
template<>
std::string encode<DigitalIOState>(const DigitalIOState& obj) {
  proto::StateMessage message;
  *message.mutable_digital_io_state() = encoder(obj);
  return message.SerializeAsString();
}
template<>
DigitalIOState decode(const std::string& msg) {
  DigitalIOState obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a DigitalIOState");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, DigitalIOState& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kDigitalIoState)) {
      return false;
    }

    auto state = message.digital_io_state();
    obj = DigitalIOState(state.state().name(), decoder(state.io_names()));
    if (!state.state().empty()) {
      obj.set_data(decoder(state.values()));
    };
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *      SpatialState
 * ---------------------- */
template<>
std::string encode<SpatialState>(const SpatialState& obj);
template<>
SpatialState decode(const std::string& msg);
template<>
bool decode(const std::string& msg, SpatialState& obj);
template<>
std::string encode<SpatialState>(const SpatialState& obj) {
  proto::StateMessage message;
  *message.mutable_spatial_state() = encoder(obj);
  return message.SerializeAsString();
}
template<>
SpatialState decode(const std::string& msg) {
  SpatialState obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a SpatialState");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, SpatialState& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kSpatialState)) {
      return false;
    }

    auto spatial_state = message.spatial_state();
    obj = SpatialState(spatial_state.state().name(), spatial_state.reference_frame());

    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *     CartesianState
 * ---------------------- */
template<>
std::string encode<CartesianState>(const CartesianState& obj);
template<>
CartesianState decode(const std::string& msg);
template<>
bool decode(const std::string& msg, CartesianState& obj);
template<>
std::string encode<CartesianState>(const CartesianState& obj) {
  proto::StateMessage message;
  *message.mutable_cartesian_state() = encoder(obj);
  return message.SerializeAsString();
}
template<>
CartesianState decode(const std::string& msg) {
  CartesianState obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a CartesianState");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, CartesianState& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kCartesianState)) {
      return false;
    }

    auto state = message.cartesian_state();
    obj = CartesianState(state.spatial_state().state().name(), state.spatial_state().reference_frame());
    if (!state.spatial_state().state().empty()) {
      obj.set_position(decoder(state.position()));
      obj.set_orientation(decoder(state.orientation()));
      obj.set_linear_velocity(decoder(state.linear_velocity()));
      obj.set_angular_velocity(decoder(state.angular_velocity()));
      obj.set_linear_acceleration(decoder(state.linear_acceleration()));
      obj.set_angular_acceleration(decoder(state.angular_acceleration()));
      obj.set_force(decoder(state.force()));
      obj.set_torque(decoder(state.torque()));
    }
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *     CartesianPose
 * ---------------------- */
template<>
std::string encode<CartesianPose>(const CartesianPose& obj);
template<>
CartesianPose decode(const std::string& msg);
template<>
bool decode(const std::string& msg, CartesianPose& obj);
template<>
std::string encode<CartesianPose>(const CartesianPose& obj) {
  proto::StateMessage message;
  auto cartesian_state = encoder(static_cast<CartesianState>(obj));
  *message.mutable_cartesian_pose()->mutable_spatial_state() = cartesian_state.spatial_state();
  if (!cartesian_state.spatial_state().state().empty()) {
    *message.mutable_cartesian_pose()->mutable_position() = cartesian_state.position();
    *message.mutable_cartesian_pose()->mutable_orientation() = cartesian_state.orientation();
  }
  return message.SerializeAsString();
}
template<>
CartesianPose decode(const std::string& msg) {
  CartesianPose obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a CartesianPose");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, CartesianPose& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kCartesianPose)) {
      return false;
    }
    auto pose = message.cartesian_pose();
    obj = CartesianPose(pose.spatial_state().state().name(), pose.spatial_state().reference_frame());
    if (!pose.spatial_state().state().empty()) {
      obj.set_position(decoder(pose.position()));
      obj.set_orientation(decoder(pose.orientation()));
    }
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *     CartesianTwist
 * ---------------------- */
template<>
std::string encode<CartesianTwist>(const CartesianTwist& obj);
template<>
CartesianTwist decode(const std::string& msg);
template<>
bool decode(const std::string& msg, CartesianTwist& obj);
template<>
std::string encode<CartesianTwist>(const CartesianTwist& obj) {
  proto::StateMessage message;
  auto cartesian_state = encoder(static_cast<CartesianState>(obj));
  *message.mutable_cartesian_twist()->mutable_spatial_state() = cartesian_state.spatial_state();
  if (!cartesian_state.spatial_state().state().empty()) {
    *message.mutable_cartesian_twist()->mutable_linear_velocity() = cartesian_state.linear_velocity();
    *message.mutable_cartesian_twist()->mutable_angular_velocity() = cartesian_state.angular_velocity();
  }
  return message.SerializeAsString();
}
template<>
CartesianTwist decode(const std::string& msg) {
  CartesianTwist obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a CartesianTwist");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, CartesianTwist& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kCartesianTwist)) {
      return false;
    }
    auto twist = message.cartesian_twist();
    obj = CartesianTwist(twist.spatial_state().state().name(), twist.spatial_state().reference_frame());
    if (!twist.spatial_state().state().empty()) {
      obj.set_linear_velocity(decoder(twist.linear_velocity()));
      obj.set_angular_velocity(decoder(twist.angular_velocity()));
    }
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *  CartesianAcceleration
 * ---------------------- */
template<>
std::string encode<CartesianAcceleration>(const CartesianAcceleration& obj);
template<>
CartesianAcceleration decode(const std::string& msg);
template<>
bool decode(const std::string& msg, CartesianAcceleration& obj);
template<>
std::string encode<CartesianAcceleration>(const CartesianAcceleration& obj) {
  proto::StateMessage message;
  auto cartesian_state = encoder(static_cast<CartesianState>(obj));
  *message.mutable_cartesian_acceleration()->mutable_spatial_state() = cartesian_state.spatial_state();
  if (!cartesian_state.spatial_state().state().empty()) {
    *message.mutable_cartesian_acceleration()->mutable_linear_acceleration() = cartesian_state.linear_acceleration();
    *message.mutable_cartesian_acceleration()->mutable_angular_acceleration() = cartesian_state.angular_acceleration();
  }
  return message.SerializeAsString();
}
template<>
CartesianAcceleration decode(const std::string& msg) {
  CartesianAcceleration obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a CartesianAcceleration");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, CartesianAcceleration& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kCartesianAcceleration)) {
      return false;
    }
    auto acceleration = message.cartesian_acceleration();
    obj = CartesianAcceleration(
        acceleration.spatial_state().state().name(), acceleration.spatial_state().reference_frame()
    );
    if (!acceleration.spatial_state().state().empty()) {
      obj.set_linear_acceleration(decoder(acceleration.linear_acceleration()));
      obj.set_angular_acceleration(decoder(acceleration.angular_acceleration()));
    }
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *    CartesianWrench
 * ---------------------- */
template<>
std::string encode<CartesianWrench>(const CartesianWrench& obj);
template<>
CartesianWrench decode(const std::string& msg);
template<>
bool decode(const std::string& msg, CartesianWrench& obj);
template<>
std::string encode<CartesianWrench>(const CartesianWrench& obj) {
  proto::StateMessage message;
  auto cartesian_state = encoder(static_cast<CartesianState>(obj));
  *message.mutable_cartesian_wrench()->mutable_spatial_state() = cartesian_state.spatial_state();
  if (!cartesian_state.spatial_state().state().empty()) {
    *message.mutable_cartesian_wrench()->mutable_force() = cartesian_state.force();
    *message.mutable_cartesian_wrench()->mutable_torque() = cartesian_state.torque();
  }
  return message.SerializeAsString();
}
template<>
CartesianWrench decode(const std::string& msg) {
  CartesianWrench obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a CartesianWrench");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, CartesianWrench& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kCartesianWrench)) {
      return false;
    }
    auto wrench = message.cartesian_wrench();
    obj = CartesianWrench(wrench.spatial_state().state().name(), wrench.spatial_state().reference_frame());
    if (!wrench.spatial_state().state().empty()) {
      obj.set_force(decoder(wrench.force()));
      obj.set_torque(decoder(wrench.torque()));
    }
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *        Jacobian
 * ---------------------- */
template<>
std::string encode<Jacobian>(const Jacobian& obj);
template<>
Jacobian decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Jacobian& obj);
template<>
std::string encode<Jacobian>(const Jacobian& obj) {
  proto::StateMessage message;
  *message.mutable_jacobian() = encoder(obj);
  return message.SerializeAsString();
}
template<>
Jacobian decode(const std::string& msg) {
  Jacobian obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a Jacobian");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, Jacobian& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg) && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJacobian
        )) {
      return false;
    }

    auto jacobian = message.jacobian();
    obj = Jacobian(
        jacobian.state().name(), decoder(jacobian.joint_names()), jacobian.frame(), jacobian.reference_frame()
    );
    if (!jacobian.state().empty() && !jacobian.data().empty()) {
      auto raw_data = const_cast<double*>(jacobian.data().data());
      auto data = Eigen::Map<Eigen::MatrixXd>(raw_data, jacobian.rows(), jacobian.cols());
      obj.set_data(data);
    }
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *       JointState
 * ---------------------- */
template<>
std::string encode<JointState>(const JointState& obj);
template<>
JointState decode(const std::string& msg);
template<>
bool decode(const std::string& msg, JointState& obj);
template<>
std::string encode<JointState>(const JointState& obj) {
  proto::StateMessage message;
  *message.mutable_joint_state() = encoder(obj);
  return message.SerializeAsString();
}
template<>
JointState decode(const std::string& msg) {
  JointState obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a JointState");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, JointState& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJointState)) {
      return false;
    }

    auto state = message.joint_state();
    obj = JointState(state.state().name(), decoder(state.joint_names()));
    if (!state.state().empty()) {
      obj.set_positions(decoder(state.positions()));
      obj.set_velocities(decoder(state.velocities()));
      obj.set_accelerations(decoder(state.accelerations()));
      obj.set_torques(decoder(state.torques()));
    };
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *     JointPositions
 * ---------------------- */
template<>
std::string encode<JointPositions>(const JointPositions& obj);
template<>
JointPositions decode(const std::string& msg);
template<>
bool decode(const std::string& msg, JointPositions& obj);
template<>
std::string encode<JointPositions>(const JointPositions& obj) {
  proto::StateMessage message;
  auto joint_state = encoder(static_cast<JointState>(obj));
  *message.mutable_joint_positions()->mutable_state() = joint_state.state();
  *message.mutable_joint_positions()->mutable_joint_names() = joint_state.joint_names();
  if (!joint_state.state().empty()) {
    *message.mutable_joint_positions()->mutable_positions() = joint_state.positions();
  }
  return message.SerializeAsString();
}
template<>
JointPositions decode(const std::string& msg) {
  JointPositions obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a JointPositions");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, JointPositions& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJointPositions)) {
      return false;
    }

    auto positions = message.joint_positions();
    obj = JointPositions(positions.state().name(), decoder(positions.joint_names()));
    if (!positions.state().empty()) {
      obj.set_positions(decoder(positions.positions()));
    }
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *    JointVelocities
 * ---------------------- */
template<>
std::string encode<JointVelocities>(const JointVelocities& obj);
template<>
JointVelocities decode(const std::string& msg);
template<>
bool decode(const std::string& msg, JointVelocities& obj);
template<>
std::string encode<JointVelocities>(const JointVelocities& obj) {
  proto::StateMessage message;
  auto joint_state = encoder(static_cast<JointState>(obj));
  *message.mutable_joint_velocities()->mutable_state() = joint_state.state();
  *message.mutable_joint_velocities()->mutable_joint_names() = joint_state.joint_names();
  if (!joint_state.state().empty()) {
    *message.mutable_joint_velocities()->mutable_velocities() = joint_state.velocities();
  }
  return message.SerializeAsString();
}
template<>
JointVelocities decode(const std::string& msg) {
  JointVelocities obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a JointVelocities");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, JointVelocities& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJointVelocities)) {
      return false;
    }

    auto velocities = message.joint_velocities();
    obj = JointVelocities(velocities.state().name(), decoder(velocities.joint_names()));
    if (!velocities.state().empty()) {
      obj.set_velocities(decoder(velocities.velocities()));
    }
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *   JointAccelerations
 * ---------------------- */
template<>
std::string encode<JointAccelerations>(const JointAccelerations& obj);
template<>
JointAccelerations decode(const std::string& msg);
template<>
bool decode(const std::string& msg, JointAccelerations& obj);
template<>
std::string encode<JointAccelerations>(const JointAccelerations& obj) {
  proto::StateMessage message;
  auto joint_state = encoder(static_cast<JointState>(obj));
  *message.mutable_joint_accelerations()->mutable_state() = joint_state.state();
  *message.mutable_joint_accelerations()->mutable_joint_names() = joint_state.joint_names();
  if (!joint_state.state().empty()) {
    *message.mutable_joint_accelerations()->mutable_accelerations() = joint_state.accelerations();
  }
  return message.SerializeAsString();
}
template<>
JointAccelerations decode(const std::string& msg) {
  JointAccelerations obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a JointAccelerations");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, JointAccelerations& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJointAccelerations)) {
      return false;
    }

    auto accelerations = message.joint_accelerations();
    obj = JointAccelerations(accelerations.state().name(), decoder(accelerations.joint_names()));
    if (!accelerations.state().empty()) {
      obj.set_accelerations(decoder(accelerations.accelerations()));
    }
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *      JointTorques
 * ---------------------- */
template<>
std::string encode<JointTorques>(const JointTorques& obj);
template<>
JointTorques decode(const std::string& msg);
template<>
bool decode(const std::string& msg, JointTorques& obj);
template<>
std::string encode<JointTorques>(const JointTorques& obj) {
  proto::StateMessage message;
  auto joint_state = encoder(static_cast<JointState>(obj));
  *message.mutable_joint_torques()->mutable_state() = joint_state.state();
  *message.mutable_joint_torques()->mutable_joint_names() = joint_state.joint_names();
  if (!joint_state.state().empty()) {
    *message.mutable_joint_torques()->mutable_torques() = joint_state.torques();
  }
  return message.SerializeAsString();
}
template<>
JointTorques decode(const std::string& msg) {
  JointTorques obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a JointTorques");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, JointTorques& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kJointTorques)) {
      return false;
    }

    auto torques = message.joint_torques();
    obj = JointTorques(torques.state().name(), decoder(torques.joint_names()));
    if (!torques.state().empty()) {
      obj.set_torques(decoder(torques.torques()));
    }
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *      Parameter<T>
 * ---------------------- */
template<typename T>
static std::string encode_parameter(const Parameter<T>& obj);
template<typename T>
static Parameter<T> decode_parameter(const std::string& msg);
template<typename T>
static bool decode_parameter(const std::string& msg, Parameter<T>& obj);

template<typename T>
static std::string encode_parameter(const Parameter<T>& obj) {
  proto::StateMessage message;
  *message.mutable_parameter() = encoder<T>(obj);
  return message.SerializeAsString();
}
template<typename T>
static Parameter<T> decode_parameter(const std::string& msg) {
  Parameter<T> obj("");
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a Parameter");
  }
  return obj;
}
template<typename T>
static bool decode_parameter(const std::string& msg, Parameter<T>& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg)
          && message.message_type_case() == proto::StateMessage::MessageTypeCase::kParameter)) {
      return false;
    }
    obj = decoder<T>(message.parameter());
    return true;
  } catch (...) {
    return false;
  }
}

/* ----------------------
 *          INT
 * ---------------------- */
template<>
std::string encode<Parameter<int>>(const Parameter<int>& obj);
template<>
Parameter<int> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<int>& obj);
template<>
std::string encode<Parameter<int>>(const Parameter<int>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<int> decode(const std::string& msg) {
  return decode_parameter<int>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<int>& obj) {
  return decode_parameter(msg, obj);
}

/* ----------------------
 *       INT_ARRAY
 * ---------------------- */
template<>
std::string encode<Parameter<std::vector<int>>>(const Parameter<std::vector<int>>& obj);
template<>
Parameter<std::vector<int>> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<std::vector<int>>& obj);
template<>
std::string encode<Parameter<std::vector<int>>>(const Parameter<std::vector<int>>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<std::vector<int>> decode(const std::string& msg) {
  return decode_parameter<std::vector<int>>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<std::vector<int>>& obj) {
  return decode_parameter(msg, obj);
}

/* ----------------------
 *        DOUBLE
 * ---------------------- */
template<>
std::string encode<Parameter<double>>(const Parameter<double>& obj);
template<>
Parameter<double> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<double>& obj);
template<>
std::string encode<Parameter<double>>(const Parameter<double>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<double> decode(const std::string& msg) {
  return decode_parameter<double>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<double>& obj) {
  return decode_parameter(msg, obj);
}

/* ----------------------
 *      DOUBLE_ARRAY
 * ---------------------- */
template<>
std::string encode<Parameter<std::vector<double>>>(const Parameter<std::vector<double>>& obj);
template<>
Parameter<std::vector<double>> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<std::vector<double>>& obj);
template<>
std::string encode<Parameter<std::vector<double>>>(const Parameter<std::vector<double>>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<std::vector<double>> decode(const std::string& msg) {
  return decode_parameter<std::vector<double>>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<std::vector<double>>& obj) {
  return decode_parameter(msg, obj);
}

/* ----------------------
 *          BOOL
 * ---------------------- */
template<>
std::string encode<Parameter<bool>>(const Parameter<bool>& obj);
template<>
Parameter<bool> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<bool>& obj);
template<>
std::string encode<Parameter<bool>>(const Parameter<bool>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<bool> decode(const std::string& msg) {
  return decode_parameter<bool>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<bool>& obj) {
  return decode_parameter(msg, obj);
}

/* ----------------------
 *       BOOL_ARRAY
 * ---------------------- */
template<>
std::string encode<Parameter<std::vector<bool>>>(const Parameter<std::vector<bool>>& obj);
template<>
Parameter<std::vector<bool>> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<std::vector<bool>>& obj);
template<>
std::string encode<Parameter<std::vector<bool>>>(const Parameter<std::vector<bool>>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<std::vector<bool>> decode(const std::string& msg) {
  return decode_parameter<std::vector<bool>>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<std::vector<bool>>& obj) {
  return decode_parameter(msg, obj);
}

/* ----------------------
 *         STRING
 * ---------------------- */
template<>
std::string encode<Parameter<std::string>>(const Parameter<std::string>& obj);
template<>
Parameter<std::string> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<std::string>& obj);
template<>
std::string encode<Parameter<std::string>>(const Parameter<std::string>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<std::string> decode(const std::string& msg) {
  return decode_parameter<std::string>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<std::string>& obj) {
  return decode_parameter(msg, obj);
}

/* ----------------------
 *      STRING_ARRAY
 * ---------------------- */
template<>
std::string encode<Parameter<std::vector<std::string>>>(const Parameter<std::vector<std::string>>& obj);
template<>
Parameter<std::vector<std::string>> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<std::vector<std::string>>& obj);
template<>
std::string encode<Parameter<std::vector<std::string>>>(const Parameter<std::vector<std::string>>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<std::vector<std::string>> decode(const std::string& msg) {
  return decode_parameter<std::vector<std::string>>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<std::vector<std::string>>& obj) {
  return decode_parameter(msg, obj);
}

/* ----------------------
 *         VECTOR
 * ---------------------- */
template<>
std::string encode<Parameter<Eigen::VectorXd>>(const Parameter<Eigen::VectorXd>& obj);
template<>
Parameter<Eigen::VectorXd> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<Eigen::VectorXd>& obj);
template<>
std::string encode<Parameter<Eigen::VectorXd>>(const Parameter<Eigen::VectorXd>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<Eigen::VectorXd> decode(const std::string& msg) {
  return decode_parameter<Eigen::VectorXd>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<Eigen::VectorXd>& obj) {
  return decode_parameter(msg, obj);
}

/* ----------------------
 *         MATRIX
 * ---------------------- */
template<>
std::string encode<Parameter<Eigen::MatrixXd>>(const Parameter<Eigen::MatrixXd>& obj);
template<>
Parameter<Eigen::MatrixXd> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<Eigen::MatrixXd>& obj);
template<>
std::string encode<Parameter<Eigen::MatrixXd>>(const Parameter<Eigen::MatrixXd>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<Eigen::MatrixXd> decode(const std::string& msg) {
  return decode_parameter<Eigen::MatrixXd>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<Eigen::MatrixXd>& obj) {
  return decode_parameter(msg, obj);
}

/*-----------------------
 * STD::SHARED_PTR<STATE>
 * ---------------------- */
template<typename T>
std::shared_ptr<T> safe_dynamic_pointer_cast(const std::shared_ptr<State>& state) {
  auto new_state = std::dynamic_pointer_cast<T>(state);
  if (new_state == nullptr) {
    throw std::invalid_argument("Dynamic pointer casting of state failed.");
  }
  return new_state;
}

template<>
std::string encode<std::shared_ptr<State>>(const std::shared_ptr<State>& obj);
template<>
std::shared_ptr<State> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, std::shared_ptr<State>& obj);
template<>
std::string encode<std::shared_ptr<State>>(const std::shared_ptr<State>& obj) {
  std::string message;
  switch (obj->get_type()) {
    case StateType::STATE:
      message = encode<State>(*obj);
      break;
    case StateType::DIGITAL_IO_STATE:
      message = encode<DigitalIOState>(*safe_dynamic_pointer_cast<DigitalIOState>(obj));
      break;
    case StateType::ANALOG_IO_STATE:
      message = encode<AnalogIOState>(*safe_dynamic_pointer_cast<AnalogIOState>(obj));
      break;
    case StateType::SPATIAL_STATE:
      message = encode<SpatialState>(*safe_dynamic_pointer_cast<SpatialState>(obj));
      break;
    case StateType::CARTESIAN_STATE:
      message = encode<CartesianState>(*safe_dynamic_pointer_cast<CartesianState>(obj));
      break;
    case StateType::CARTESIAN_POSE:
      message = encode<CartesianPose>(*safe_dynamic_pointer_cast<CartesianPose>(obj));
      break;
    case StateType::CARTESIAN_TWIST:
      message = encode<CartesianTwist>(*safe_dynamic_pointer_cast<CartesianTwist>(obj));
      break;
    case StateType::CARTESIAN_ACCELERATION:
      message = encode<CartesianAcceleration>(*safe_dynamic_pointer_cast<CartesianAcceleration>(obj));
      break;
    case StateType::CARTESIAN_WRENCH:
      message = encode<CartesianWrench>(*safe_dynamic_pointer_cast<CartesianWrench>(obj));
      break;
    case StateType::JOINT_STATE:
      message = encode<JointState>(*safe_dynamic_pointer_cast<JointState>(obj));
      break;
    case StateType::JOINT_POSITIONS:
      message = encode<JointPositions>(*safe_dynamic_pointer_cast<JointPositions>(obj));
      break;
    case StateType::JOINT_VELOCITIES:
      message = encode<JointVelocities>(*safe_dynamic_pointer_cast<JointVelocities>(obj));
      break;
    case StateType::JOINT_ACCELERATIONS:
      message = encode<JointAccelerations>(*safe_dynamic_pointer_cast<JointAccelerations>(obj));
      break;
    case StateType::JOINT_TORQUES:
      message = encode<JointTorques>(*safe_dynamic_pointer_cast<JointTorques>(obj));
      break;
    case StateType::JACOBIAN:
      message = encode<Jacobian>(*safe_dynamic_pointer_cast<Jacobian>(obj));
      break;
    case StateType::PARAMETER: {
      auto param_ptr = safe_dynamic_pointer_cast<ParameterInterface>(obj);
      switch (param_ptr->get_parameter_type()) {
        case ParameterType::BOOL:
          message = encode<Parameter<bool>>(*safe_dynamic_pointer_cast<Parameter<bool>>(param_ptr));
          break;
        case ParameterType::BOOL_ARRAY:
          message =
              encode<Parameter<std::vector<bool>>>(*safe_dynamic_pointer_cast<Parameter<std::vector<bool>>>(param_ptr));
          break;
        case ParameterType::INT:
          message = encode<Parameter<int>>(*safe_dynamic_pointer_cast<Parameter<int>>(param_ptr));
          break;
        case ParameterType::INT_ARRAY:
          message =
              encode<Parameter<std::vector<int>>>(*safe_dynamic_pointer_cast<Parameter<std::vector<int>>>(param_ptr));
          break;
        case ParameterType::DOUBLE:
          message = encode<Parameter<double>>(*safe_dynamic_pointer_cast<Parameter<double>>(param_ptr));
          break;
        case ParameterType::DOUBLE_ARRAY:
          message = encode<Parameter<std::vector<double>>>(
              *safe_dynamic_pointer_cast<Parameter<std::vector<double>>>(param_ptr)
          );
          break;
        case ParameterType::STRING:
          message = encode<Parameter<std::string>>(*safe_dynamic_pointer_cast<Parameter<std::string>>(param_ptr));
          break;
        case ParameterType::STRING_ARRAY:
          message = encode<Parameter<std::vector<std::string>>>(
              *safe_dynamic_pointer_cast<Parameter<std::vector<std::string>>>(param_ptr)
          );
          break;
        case ParameterType::VECTOR:
          message =
              encode<Parameter<Eigen::VectorXd>>(*safe_dynamic_pointer_cast<Parameter<Eigen::VectorXd>>(param_ptr));
          break;
        case ParameterType::MATRIX:
          message =
              encode<Parameter<Eigen::MatrixXd>>(*safe_dynamic_pointer_cast<Parameter<Eigen::MatrixXd>>(param_ptr));
          break;
        default:
          throw std::invalid_argument(
              "The ParameterType contained by parameter " + param_ptr->get_name() + " is unsupported."
          );
          break;
      }
      break;
    }
    default:
      throw std::invalid_argument("The StateType contained by state " + obj->get_name() + " is unsupported.");
      break;
  }
  return message;
}
template<>
std::shared_ptr<State> decode(const std::string& msg) {
  std::shared_ptr<State> obj;
  switch (check_message_type(msg)) {
    case MessageType::STATE_MESSAGE:
      obj = make_shared_state(State());
      break;
    case MessageType::DIGITAL_IO_STATE_MESSAGE:
      obj = make_shared_state(DigitalIOState());
      break;
    case MessageType::ANALOG_IO_STATE_MESSAGE:
      obj = make_shared_state(AnalogIOState());
      break;
    case MessageType::SPATIAL_STATE_MESSAGE:
      obj = make_shared_state(SpatialState());
      break;
    case MessageType::CARTESIAN_STATE_MESSAGE:
      obj = make_shared_state(CartesianState());
      break;
    case MessageType::CARTESIAN_POSE_MESSAGE:
      obj = make_shared_state(CartesianPose());
      break;
    case MessageType::CARTESIAN_TWIST_MESSAGE:
      obj = make_shared_state(CartesianTwist());
      break;
    case MessageType::CARTESIAN_ACCELERATION_MESSAGE:
      obj = make_shared_state(CartesianAcceleration());
      break;
    case MessageType::CARTESIAN_WRENCH_MESSAGE:
      obj = make_shared_state(CartesianWrench());
      break;
    case MessageType::JOINT_STATE_MESSAGE:
      obj = make_shared_state(JointState());
      break;
    case MessageType::JOINT_POSITIONS_MESSAGE:
      obj = make_shared_state(JointPositions());
      break;
    case MessageType::JOINT_VELOCITIES_MESSAGE:
      obj = make_shared_state(JointVelocities());
      break;
    case MessageType::JOINT_ACCELERATIONS_MESSAGE:
      obj = make_shared_state(JointAccelerations());
      break;
    case MessageType::JOINT_TORQUES_MESSAGE:
      obj = make_shared_state(JointTorques());
      break;
    case MessageType::JACOBIAN_MESSAGE:
      obj = make_shared_state(Jacobian());
      break;
    case MessageType::PARAMETER_MESSAGE: {
      switch (check_parameter_message_type(msg)) {
        case ParameterMessageType::BOOL:
          obj = make_shared_state(Parameter<bool>(""));
          break;
        case ParameterMessageType::BOOL_ARRAY:
          obj = make_shared_state(Parameter<std::vector<bool>>(""));
          break;
        case ParameterMessageType::INT:
          obj = make_shared_state(Parameter<int>(""));
          break;
        case ParameterMessageType::INT_ARRAY:
          obj = make_shared_state(Parameter<std::vector<int>>(""));
          break;
        case ParameterMessageType::DOUBLE:
          obj = make_shared_state(Parameter<double>(""));
          break;
        case ParameterMessageType::DOUBLE_ARRAY:
          obj = make_shared_state(Parameter<std::vector<double>>(""));
          break;
        case ParameterMessageType::STRING:
          obj = make_shared_state(Parameter<std::string>(""));
          break;
        case ParameterMessageType::STRING_ARRAY:
          obj = make_shared_state(Parameter<std::vector<std::string>>(""));
          break;
        case ParameterMessageType::VECTOR:
          obj = make_shared_state(Parameter<Eigen::VectorXd>(""));
          break;
        case ParameterMessageType::MATRIX:
          obj = make_shared_state(Parameter<Eigen::MatrixXd>(""));
          break;
        default:
          throw std::invalid_argument("The ParameterMessageType contained by this message is unsupported.");
          break;
      }
      break;
    }
    default:
      throw std::invalid_argument("The MessageType contained by this message is unsupported.");
      break;
  }
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a std::shared_ptr<State>");
  }
  return obj;
}
template<>
bool decode(const std::string& msg, std::shared_ptr<State>& obj) {
  try {
    switch (obj->get_type()) {
      case StateType::STATE:
        obj = make_shared_state(decode<State>(msg));
        break;
      case StateType::DIGITAL_IO_STATE:
        obj = make_shared_state(decode<DigitalIOState>(msg));
        break;
      case StateType::ANALOG_IO_STATE:
        obj = make_shared_state(decode<AnalogIOState>(msg));
        break;
      case StateType::SPATIAL_STATE:
        obj = make_shared_state(decode<SpatialState>(msg));
        break;
      case StateType::CARTESIAN_STATE:
        obj = make_shared_state(decode<CartesianState>(msg));
        break;
      case StateType::CARTESIAN_POSE:
        obj = make_shared_state(decode<CartesianPose>(msg));
        break;
      case StateType::CARTESIAN_TWIST:
        obj = make_shared_state(decode<CartesianTwist>(msg));
        break;
      case StateType::CARTESIAN_ACCELERATION:
        obj = make_shared_state(decode<CartesianAcceleration>(msg));
        break;
      case StateType::CARTESIAN_WRENCH:
        obj = make_shared_state(decode<CartesianWrench>(msg));
        break;
      case StateType::JOINT_STATE:
        obj = make_shared_state(decode<JointState>(msg));
        break;
      case StateType::JOINT_POSITIONS:
        obj = make_shared_state(decode<JointPositions>(msg));
        break;
      case StateType::JOINT_VELOCITIES:
        obj = make_shared_state(decode<JointVelocities>(msg));
        break;
      case StateType::JOINT_ACCELERATIONS:
        obj = make_shared_state(decode<JointAccelerations>(msg));
        break;
      case StateType::JOINT_TORQUES:
        obj = make_shared_state(decode<JointTorques>(msg));
        break;
      case StateType::JACOBIAN:
        obj = make_shared_state(decode<Jacobian>(msg));
        break;
      case StateType::PARAMETER: {
        auto param_ptr = safe_dynamic_pointer_cast<ParameterInterface>(obj);
        switch (param_ptr->get_parameter_type()) {
          case ParameterType::BOOL:
            obj = make_shared_state(decode<Parameter<bool>>(msg));
            break;
          case ParameterType::BOOL_ARRAY:
            obj = make_shared_state(decode<Parameter<std::vector<bool>>>(msg));
            break;
          case ParameterType::INT:
            obj = make_shared_state(decode<Parameter<int>>(msg));
            break;
          case ParameterType::INT_ARRAY:
            obj = make_shared_state(decode<Parameter<std::vector<int>>>(msg));
            break;
          case ParameterType::DOUBLE:
            obj = make_shared_state(decode<Parameter<double>>(msg));
            break;
          case ParameterType::DOUBLE_ARRAY:
            obj = make_shared_state(decode<Parameter<std::vector<double>>>(msg));
            break;
          case ParameterType::STRING:
            obj = make_shared_state(decode<Parameter<std::string>>(msg));
            break;
          case ParameterType::STRING_ARRAY:
            obj = make_shared_state(decode<Parameter<std::vector<std::string>>>(msg));
            break;
          case ParameterType::VECTOR:
            obj = make_shared_state(decode<Parameter<Eigen::VectorXd>>(msg));
            break;
          case ParameterType::MATRIX:
            obj = make_shared_state(decode<Parameter<Eigen::MatrixXd>>(msg));
            break;
          default:
            throw std::invalid_argument(
                "The ParameterType contained by parameter " + param_ptr->get_name() + " is unsupported."
            );
            break;
        }
        break;
      }
      default:
        throw std::invalid_argument("The StateType contained by state " + obj->get_name() + " is unsupported.");
        break;
    }
    return true;
  } catch (...) {
    return false;
  }
}

// Generic template code for future types:
/* ----------------------
 *        __TYPE__
 * ---------------------- */ /*
template<> std::string encode<__TYPE__>(const __TYPE__& obj);
template<> __TYPE__ decode(const std::string& msg);
template<> bool decode(const std::string& msg, __TYPE__& obj);
template<> std::string encode<__TYPE__>(const __TYPE__& obj) {
  proto::StateMessage message;
  // encode
  return message.SerializeAsString();
}
template<> __TYPE__ decode(const std::string& msg) {
  __TYPE__ obj;
  if (!decode(msg, obj)) {
    throw DecodingException("Could not decode the message into a __TYPE__");
  }
  return obj;
}
template<> bool decode(const std::string& msg, __TYPE__& obj) {
  try {
    proto::StateMessage message;
    if (!(message.ParseFromString(msg) && message.message_type_case() == proto::StateMessage::MessageTypeCase::k__TYPE__)) {
      return false;
    }
    // decode
    return true;
  } catch (...) {
    return false;
  }
}
*/

/* ----------------------
 *   Parameter<ParamT>
 * ---------------------- */ /*
template<>
std::string encode<Parameter<ParamT>>(const Parameter<ParamT>& obj);
template<>
Parameter<ParamT> decode(const std::string& msg);
template<>
bool decode(const std::string& msg, Parameter<ParamT>& obj);
template<>
std::string encode<Parameter<ParamT>>(const Parameter<ParamT>& obj) {
  return encode_parameter(obj);
}
template<>
Parameter<ParamT> decode(const std::string& msg) {
  return decode_parameter<ParamT>(msg);
}
template<>
bool decode(const std::string& msg, Parameter<ParamT>& obj) {
  return decode_parameter(msg, obj);
}
*/

}// namespace clproto
