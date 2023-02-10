import numpy as np
import pytest
import state_representation as sr


class Helpers:
    @staticmethod
    def assert_cartesian_equal(state1: sr.CartesianState, state2: sr.CartesianState, tolerance=1e-4,
                               test_state_attributes=True):
        if test_state_attributes:
            assert state1.get_name() == state2.get_name()
            assert state1.get_type() == state2.get_type()
            assert state1.is_empty() == state2.is_empty()
        assert state1.get_reference_frame() == state2.get_reference_frame()
        if state1:
            assert state1.dist(state2) < tolerance

    @staticmethod
    def assert_jacobian_equal(state1: sr.Jacobian, state2: sr.Jacobian, tolerance=1e-4, test_state_attributes=True):
        if test_state_attributes:
            assert state1.get_name() == state2.get_name()
            assert state1.get_type() == state2.get_type()
            assert state1.is_empty() == state2.is_empty()
        assert state1.get_frame() == state2.get_frame()
        assert state1.get_reference_frame() == state2.get_reference_frame()
        assert all([i == j for i, j in zip(state1.get_joint_names(), state2.get_joint_names())])
        if state1:
            assert state1.cols() == state2.cols()
            assert state1.rows() == state2.rows()
            np.testing.assert_array_equal(state1.data(), state2.data(), tolerance)

    @staticmethod
    def assert_joint_equal(state1: sr.JointState, state2: sr.JointState, tolerance=1e-4, test_state_attributes=True):
        if test_state_attributes:
            assert state1.get_name() == state2.get_name()
            assert state1.get_type() == state2.get_type()
            assert state1.is_empty() == state2.is_empty()
        assert all([i == j for i, j in zip(state1.get_names(), state2.get_names())])
        if state1:
            assert state1.dist(state2) < tolerance

    def assert_parameter_equal(self, state1: sr.Parameter, state2: sr.Parameter, tolerance=1e-4, test_state_attributes=True):
        if test_state_attributes:
            assert state1.get_name() == state2.get_name()
            assert state1.get_type() == state2.get_type()
            assert state1.is_empty() == state2.is_empty()
        assert state1.get_parameter_type() == state2.get_parameter_type()
        if state1.get_parameter_type() == sr.ParameterType.STATE:
            assert state1.get_parameter_state_type() == state2.get_parameter_state_type()
        if not state1.is_empty():
            if state1.get_parameter_type() == sr.ParameterType.STATE:
                if state1.get_parameter_state_type() in [sr.StateType.CARTESIAN_STATE, sr.StateType.CARTESIAN_POSE]:
                    self.assert_cartesian_equal(state1.get_value(), state2.get_value(), tolerance=tolerance)
                elif state1.get_parameter_state_type() in [sr.StateType.JOINT_STATE, sr.StateType.JOINT_POSITIONS]:
                    self.assert_joint_equal(state1.get_value(), state2.get_value(), tolerance=tolerance)
                else:
                    assert None
            elif isinstance(state1.get_value(), list):
                assert len(state1.get_value()) == len(state2.get_value())
                assert all([type(i) == type(j) for i, j in zip(state1.get_value(), state2.get_value())])
                assert all([i == j for i, j in zip(state1.get_value(), state2.get_value())])
            elif isinstance(state1.get_value(), np.ndarray):
                np.testing.assert_array_equal(state1.get_value(), state2.get_value(), tolerance)
            else:
                assert type(state1.get_value()) == type(state2.get_value())
                assert state1.get_value() == state2.get_value()

    def assert_state_equal(self, state1: sr.State, state2: sr.State, tolerance=1e-4):
        assert state1.get_name() == state2.get_name()
        assert state1.get_type() == state2.get_type()
        assert state1.is_empty() == state2.is_empty()
        if sr.StateType.CARTESIAN_STATE.value <= state1.get_type().value <= sr.StateType.CARTESIAN_WRENCH.value:
            self.assert_cartesian_equal(state1, state2, tolerance=tolerance, test_state_attributes=False)
        elif state1.get_type().value == sr.StateType.JACOBIAN.value:
            self.assert_jacobian_equal(state1, state2, tolerance=tolerance, test_state_attributes=False)
        elif sr.StateType.JOINT_STATE.value <= state1.get_type().value <= sr.StateType.JOINT_TORQUES.value:
            self.assert_joint_equal(state1, state2, tolerance=tolerance, test_state_attributes=False)
        elif state1.get_type() == sr.StateType.PARAMETER:
            self.assert_parameter_equal(state1, state2, tolerance=tolerance, test_state_attributes=False)


@pytest.fixture
def helpers():
    return Helpers()
