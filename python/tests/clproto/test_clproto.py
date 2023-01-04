import pytest

import clproto
import numpy as np
import state_representation as sr
from numpy.testing import assert_array_equal

parameters = [("int", 1, sr.ParameterType.INT, clproto.ParameterMessageType.INT),
              ("int", [1, 2, 3], sr.ParameterType.INT_ARRAY, clproto.ParameterMessageType.INT_ARRAY),
              ("double", 1.1, sr.ParameterType.DOUBLE, clproto.ParameterMessageType.DOUBLE),
              ("double", [1.1, 2.2, 3.3], sr.ParameterType.DOUBLE_ARRAY, clproto.ParameterMessageType.DOUBLE_ARRAY),
              ("bool", True, sr.ParameterType.BOOL, clproto.ParameterMessageType.BOOL),
              ("bool", [True, False, True], sr.ParameterType.BOOL_ARRAY, clproto.ParameterMessageType.BOOL_ARRAY),
              ("string", "test", sr.ParameterType.STRING, clproto.ParameterMessageType.STRING),
              ("string", ["1", "2", "3"], sr.ParameterType.STRING_ARRAY, clproto.ParameterMessageType.STRING_ARRAY),
              ("vector", np.random.rand(3), sr.ParameterType.VECTOR, clproto.ParameterMessageType.VECTOR),
              ("matrix", np.random.rand(3, 2), sr.ParameterType.MATRIX, clproto.ParameterMessageType.MATRIX)]

invalid_parameters = [("cs", sr.CartesianState().Random("test"), sr.ParameterType.STATE, sr.StateType.CARTESIAN_STATE),
                      ("pose", sr.CartesianPose().Random("test"), sr.ParameterType.STATE, sr.StateType.CARTESIAN_POSE),
                      ("js", sr.JointState().Random("test", 1), sr.ParameterType.STATE, sr.StateType.JOINT_STATE),
                      ("positions", sr.JointPositions().Random("test", 1), sr.ParameterType.STATE,
                       sr.StateType.JOINT_POSITIONS)]

states = [(sr.State(sr.StateType.STATE, "test"), clproto.MessageType.STATE_MESSAGE),
          (sr.SpatialState("test", "ref"), clproto.MessageType.SPATIAL_STATE_MESSAGE),
          (sr.CartesianState("test", "ref"), clproto.MessageType.CARTESIAN_STATE_MESSAGE),
          (sr.CartesianState().Random("test"), clproto.MessageType.CARTESIAN_STATE_MESSAGE),
          (sr.CartesianPose("test", "ref"), clproto.MessageType.CARTESIAN_POSE_MESSAGE),
          (sr.CartesianPose().Random("test"), clproto.MessageType.CARTESIAN_POSE_MESSAGE),
          (sr.CartesianTwist("test", "ref"), clproto.MessageType.CARTESIAN_TWIST_MESSAGE),
          (sr.CartesianTwist().Random("test"), clproto.MessageType.CARTESIAN_TWIST_MESSAGE),
          (sr.CartesianAcceleration("test", "ref"), clproto.MessageType.CARTESIAN_ACCELERATION_MESSAGE),
          (sr.CartesianAcceleration().Random("test"), clproto.MessageType.CARTESIAN_ACCELERATION_MESSAGE),
          (sr.CartesianWrench("test", "ref"), clproto.MessageType.CARTESIAN_WRENCH_MESSAGE),
          (sr.CartesianWrench().Random("test"), clproto.MessageType.CARTESIAN_WRENCH_MESSAGE),
          (sr.Jacobian("test", 3, "ee", "base"), clproto.MessageType.JACOBIAN_MESSAGE),
          (sr.Jacobian().Random("test", 2, "ee", "base"), clproto.MessageType.JACOBIAN_MESSAGE),
          (sr.JointState("test", 3), clproto.MessageType.JOINT_STATE_MESSAGE),
          (sr.JointState().Random("test", 2), clproto.MessageType.JOINT_STATE_MESSAGE),
          (sr.JointPositions("test", 3), clproto.MessageType.JOINT_POSITIONS_MESSAGE),
          (sr.JointPositions().Random("test", 2), clproto.MessageType.JOINT_POSITIONS_MESSAGE),
          (sr.JointVelocities("test", 3), clproto.MessageType.JOINT_VELOCITIES_MESSAGE),
          (sr.JointVelocities().Random("test", 2), clproto.MessageType.JOINT_VELOCITIES_MESSAGE),
          (sr.JointAccelerations("test", 3), clproto.MessageType.JOINT_ACCELERATIONS_MESSAGE),
          (sr.JointAccelerations().Random("test", 2), clproto.MessageType.JOINT_ACCELERATIONS_MESSAGE),
          (sr.JointTorques("test", 3), clproto.MessageType.JOINT_TORQUES_MESSAGE),
          (sr.JointTorques().Random("test", 2), clproto.MessageType.JOINT_TORQUES_MESSAGE)]


def test_pack_unpack():
    objects = [sr.CartesianState.Random("A", "B"), sr.JointState.Random("robot", 3),
               sr.Jacobian().Random("robot", 3, "robot_ee", "robot_base")]
    encoded_fields = [clproto.encode(objects[0], clproto.MessageType.CARTESIAN_STATE_MESSAGE),
                      clproto.encode(objects[1], clproto.MessageType.JOINT_STATE_MESSAGE),
                      clproto.encode(objects[2], clproto.MessageType.JACOBIAN_MESSAGE)]
    packet = clproto.pack_fields(encoded_fields)

    received_fields = clproto.unpack_fields(packet)
    assert len(received_fields) == len(encoded_fields)
    assert all([clproto.is_valid(msg) for msg in received_fields])
    decoded_objects = [clproto.decode(msg) for msg in received_fields]
    assert all([isinstance(new, type(ref)) for new, ref in zip(decoded_objects, objects)])

    # confirm cartesian state decoding
    assert objects[0].get_name() == decoded_objects[0].get_name()
    assert objects[0].get_reference_frame() == decoded_objects[0].get_reference_frame()
    assert sr.dist(objects[0], decoded_objects[0]) < 1e-3

    # confirm joint state decoding
    assert objects[1].get_name() == decoded_objects[1].get_name()
    assert all([i == j for i, j in zip(objects[1].get_names(), decoded_objects[1].get_names())])
    assert sr.dist(objects[1], decoded_objects[1]) < 1e-3

    # confirm jacobian decoding
    assert objects[2].get_name() == decoded_objects[2].get_name()
    assert objects[2].get_frame() == decoded_objects[2].get_frame()
    assert objects[2].get_reference_frame() == decoded_objects[2].get_reference_frame()
    assert all([i == j for i, j in zip(objects[2].get_joint_names(), decoded_objects[2].get_joint_names())])
    assert_array_equal(objects[2].data(), decoded_objects[2].data())


def assert_json_to_from_binary(handle, send_state, message_type, parameter_message_type=None, to_from_json=False):
    msg = clproto.encode(send_state, message_type)
    assert clproto.is_valid(msg)
    assert clproto.check_message_type(msg) == message_type
    if parameter_message_type:
        assert clproto.check_parameter_message_type(msg) == parameter_message_type

    if to_from_json:
        json = clproto.to_json(msg)
        assert len(json) > 2
        msg2 = clproto.from_json(json)
        assert clproto.is_valid(msg2)
        assert clproto.check_message_type(msg2) == message_type
        if parameter_message_type:
            assert clproto.check_parameter_message_type(msg2) == parameter_message_type
        recv_state = clproto.decode(msg2)
    else:
        recv_state = clproto.decode(msg)

    assert isinstance(recv_state, type(send_state))
    handle(send_state, recv_state)


@pytest.mark.parametrize("state,message_type", states)
def test_encode_decode_states(helpers, state, message_type):
    assert_json_to_from_binary(helpers.assert_state_equal, state, message_type)


@pytest.mark.parametrize("name,value,parameter_type,message_type", parameters)
def test_encode_decode_parameters(helpers, name, value, parameter_type, message_type):
    assert_json_to_from_binary(helpers.assert_parameter_equal, sr.Parameter(name, value, parameter_type),
                               clproto.MessageType.PARAMETER_MESSAGE, message_type)


# FIXME: empty parameters that are encoded and decoded are not empty anymore!
@pytest.mark.skip
@pytest.mark.parametrize("name,value,parameter_type,message_type", parameters)
def test_encode_decode_empty_parameters(helpers, name, value, parameter_type, message_type):
    assert_json_to_from_binary(helpers.assert_parameter_equal, sr.Parameter(name, parameter_type),
                               clproto.MessageType.PARAMETER_MESSAGE, message_type)


@pytest.mark.parametrize("name,value,parameter_type,parameter_state_type", invalid_parameters)
def test_encode_invalid_parameter(name, value, parameter_type, parameter_state_type):
    send_state = sr.Parameter(name, value, parameter_type, parameter_state_type)
    with pytest.raises(ValueError):
        clproto.encode(send_state, clproto.MessageType.PARAMETER_MESSAGE)


@pytest.mark.parametrize("state,message_type", states)
def test_json_to_from_binary(helpers, state, message_type):
    assert_json_to_from_binary(helpers.assert_state_equal, state, message_type, to_from_json=True)


@pytest.mark.parametrize("name,value,parameter_type,message_type", parameters)
def test_json_to_from_binary_parameters(helpers, name, value, parameter_type, message_type):
    assert_json_to_from_binary(helpers.assert_parameter_equal, sr.Parameter(name, value, parameter_type),
                               clproto.MessageType.PARAMETER_MESSAGE, message_type, True)


# FIXME: empty parameters that are encoded and decoded are not empty anymore!
@pytest.mark.skip
@pytest.mark.parametrize("name,value,parameter_type,message_type", parameters)
def test_json_to_from_binary_empty_parameters(helpers, name, value, parameter_type, message_type):
    assert_json_to_from_binary(helpers.assert_parameter_equal, sr.Parameter(name, parameter_type),
                               clproto.MessageType.PARAMETER_MESSAGE, message_type, True)


def test_json_string_comparison():
    msg = clproto.encode(sr.CartesianPose("A", 1.0, 0.5, 3.0, "B"), clproto.MessageType.CARTESIAN_POSE_MESSAGE)
    json = clproto.to_json(msg)
    assert json == "{\"cartesianPose\":{\"spatialState\":{\"state\":{\"name\":\"A\",\"type\":" \
                   "\"CARTESIAN_POSE\"},\"referenceFrame\":\"B\"},\"position\":{\"x\":1,\"y\":0.5," \
                   "\"z\":3},\"orientation\":{\"w\":1,\"vec\":{}}}}"

    joint_state = sr.JointState("robot", 3)
    joint_state.set_velocities([0.3, 0.1, 0.6])
    msg = clproto.encode(joint_state, clproto.MessageType.JOINT_STATE_MESSAGE)
    json = clproto.to_json(msg)
    assert json == "{\"jointState\":{\"state\":{\"name\":\"robot\",\"type\":\"JOINT_STATE\"}," \
                   "\"jointNames\":[\"joint0\",\"joint1\",\"joint2\"],\"positions\":[0,0,0]," \
                   "\"velocities\":[0.3,0.1,0.6],\"accelerations\":[0,0,0],\"torques\":[0,0,0]}}"

    msg = clproto.encode(sr.Jacobian("robot", 3, "test"), clproto.MessageType.JACOBIAN_MESSAGE)
    json = clproto.to_json(msg)
    assert json == "{\"jacobian\":{\"state\":{\"name\":\"robot\",\"type\":\"JACOBIAN\",\"empty\":true}}}"


def test_decode_invalid_string():
    dummy_msg = "hello world"
    assert not clproto.is_valid(dummy_msg)
    assert clproto.check_message_type(dummy_msg) == clproto.MessageType.UNKNOWN_MESSAGE
    # FIXME: this should rather raise a clproto.DecodingError
    recv_state = clproto.decode(dummy_msg)
    assert recv_state is None
