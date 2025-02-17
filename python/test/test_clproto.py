import pytest

import clproto
import numpy as np
import datetime
import state_representation as sr

states = [(sr.State("test"), clproto.MessageType.STATE_MESSAGE),
          (sr.DigitalIOState("test"), clproto.MessageType.DIGITAL_IO_STATE_MESSAGE),
          (sr.AnalogIOState("test"), clproto.MessageType.ANALOG_IO_STATE_MESSAGE),
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
          (sr.JointTorques().Random("test", 2), clproto.MessageType.JOINT_TORQUES_MESSAGE),
          (sr.CartesianTrajectory("trajectory", sr.CartesianState().Random("test"), datetime.timedelta(seconds=1)), 
            clproto.MessageType.CARTESIAN_TRAJECTORY_MESSAGE),
          (sr.JointTrajectory("trajectory", sr.JointState().Random("test", 25), datetime.timedelta(seconds=1)), 
            clproto.MessageType.JOINT_TRAJECTORY_MESSAGE)]

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


def test_packing(helpers):
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
    # decoded objects should be of same instance as original objects
    assert all([isinstance(new, type(ref)) for new, ref in zip(decoded_objects, objects)])

    # confirm cartesian state decoding
    helpers.assert_cartesian_equal(objects[0], decoded_objects[0])

    # confirm joint state decoding
    helpers.assert_joint_equal(objects[1], decoded_objects[1])

    # confirm jacobian decoding
    helpers.assert_jacobian_equal(objects[2], decoded_objects[2])


def assert_encode_decode(send_state, message_type, handle, parameter_message_type=None):
    msg = clproto.encode(send_state, message_type)
    assert clproto.is_valid(msg)
    assert clproto.check_message_type(msg) == message_type
    if parameter_message_type:
        assert clproto.check_parameter_message_type(msg) == parameter_message_type

    json = clproto.to_json(msg)
    assert len(json) > 2
    msg2 = clproto.from_json(json)
    assert clproto.is_valid(msg2)
    assert clproto.check_message_type(msg2) == message_type
    if parameter_message_type:
        assert clproto.check_parameter_message_type(msg2) == parameter_message_type
    recv_state = clproto.decode(msg2)

    assert isinstance(recv_state, type(send_state))
    handle(send_state, recv_state)


@pytest.mark.parametrize("state,message_type", states)
def test_encode_decode_states(helpers, state, message_type):
    assert_encode_decode(state, message_type, helpers.assert_state_equal)
    state.reset()
    assert_encode_decode(state, message_type, helpers.assert_state_equal)


@pytest.mark.parametrize("name,value,parameter_type,message_type", parameters)
def test_encode_decode_parameters(helpers, name, value, parameter_type, message_type):
    parameter = sr.Parameter(name, value, parameter_type)
    assert_encode_decode(parameter, clproto.MessageType.PARAMETER_MESSAGE, helpers.assert_parameter_equal, message_type)
    parameter.reset()
    assert_encode_decode(parameter, clproto.MessageType.PARAMETER_MESSAGE, helpers.assert_parameter_equal, message_type)


@pytest.mark.parametrize("name,value,parameter_type,parameter_state_type", invalid_parameters)
def test_encode_invalid_parameter(name, value, parameter_type, parameter_state_type):
    with pytest.raises(clproto.EncodingError):
        clproto.encode(sr.Parameter(name, value, parameter_type, parameter_state_type),
                       clproto.MessageType.PARAMETER_MESSAGE)


def test_json_string_comparison():
    msg = clproto.encode(sr.CartesianPose("A", 1.0, 0.5, 3.0, "B"), clproto.MessageType.CARTESIAN_POSE_MESSAGE)
    json = clproto.to_json(msg)
    assert json == "{\"cartesianPose\":{\"spatialState\":{\"state\":{\"name\":\"A\"},\"referenceFrame\":\"B\"}," \
                   "\"position\":{\"x\":1,\"y\":0.5,\"z\":3},\"orientation\":{\"w\":1,\"vec\":{}}}}"

    joint_state = sr.JointState("robot", 3)
    joint_state.set_velocities([0.3, 0.1, 0.6])
    msg = clproto.encode(joint_state, clproto.MessageType.JOINT_STATE_MESSAGE)
    json = clproto.to_json(msg)
    assert json == "{\"jointState\":{\"state\":{\"name\":\"robot\"}," \
                   "\"jointNames\":[\"joint0\",\"joint1\",\"joint2\"],\"positions\":[0,0,0]," \
                   "\"velocities\":[0.3,0.1,0.6],\"accelerations\":[0,0,0],\"torques\":[0,0,0]}}"

    msg = clproto.encode(sr.Jacobian("robot", 3, "test"), clproto.MessageType.JACOBIAN_MESSAGE)
    json = clproto.to_json(msg)
    assert json == "{\"jacobian\":{\"state\":{\"name\":\"robot\",\"empty\":true}," \
                   "\"jointNames\":[\"joint0\",\"joint1\",\"joint2\"],\"frame\":\"test\"," \
                   "\"referenceFrame\":\"world\",\"rows\":6,\"cols\":3}}"


def test_decode_invalid_string():
    dummy_msg = "hello world"
    assert not clproto.is_valid(dummy_msg)
    assert clproto.check_message_type(dummy_msg) == clproto.MessageType.UNKNOWN_MESSAGE
    with pytest.raises(clproto.DecodingError):
        clproto.decode(dummy_msg)

    with pytest.raises(clproto.JsonParsingError):
        clproto.from_json("")
