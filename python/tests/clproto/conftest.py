import clproto
import numpy as np
import state_representation as sr

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
