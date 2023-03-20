import pytest

import numpy as np
import state_representation as sr
from numpy.testing import assert_array_almost_equal
from ..conftest import Helpers


def assert_value_equal(value, expected_value):
    assert value == expected_value


def assert_list_equal(value, expected_value):
    assert len(value) == len(expected_value)
    assert all([a == b for a, b in zip(value, expected_value)])


def assert_numpy_equal(value, expected_value, tol=1e-4):
    assert_array_almost_equal(value, expected_value, tol)


parameters = [("int", 1, sr.ParameterType.INT, sr.StateType.NONE, assert_value_equal),
              ("int", [1, 2, 3], sr.ParameterType.INT_ARRAY, sr.StateType.NONE, assert_list_equal),
              ("double", 1.1, sr.ParameterType.DOUBLE, sr.StateType.NONE, assert_value_equal),
              ("double", [1.1, 2.2, 3.3], sr.ParameterType.DOUBLE_ARRAY, sr.StateType.NONE, assert_list_equal),
              ("bool", True, sr.ParameterType.BOOL, sr.StateType.NONE, assert_value_equal),
              ("bool", [True, False, True], sr.ParameterType.BOOL_ARRAY, sr.StateType.NONE, assert_list_equal),
              ("string", "test", sr.ParameterType.STRING, sr.StateType.NONE, assert_value_equal),
              ("string", ["1", "2", "3"], sr.ParameterType.STRING_ARRAY, sr.StateType.NONE, assert_list_equal),
              ("vector", np.random.rand(3), sr.ParameterType.VECTOR, sr.StateType.NONE, assert_numpy_equal),
              ("matrix", np.random.rand(3, 2), sr.ParameterType.MATRIX, sr.StateType.NONE, assert_numpy_equal),
              ("cart_state", sr.CartesianState().Random("test", "base"), sr.ParameterType.STATE,
               sr.StateType.CARTESIAN_STATE, Helpers().assert_state_equal),
              ("cart_pose", sr.CartesianPose().Random("test", "base"), sr.ParameterType.STATE,
               sr.StateType.CARTESIAN_POSE, Helpers().assert_state_equal),
              ("joint_state", sr.JointState().Random("test", 3), sr.ParameterType.STATE, sr.StateType.JOINT_STATE,
               Helpers().assert_state_equal),
              ("joint_pos", sr.JointPositions().Random("test", 3), sr.ParameterType.STATE,
               sr.StateType.JOINT_POSITIONS, Helpers().assert_state_equal),
              ("ellipsoid", sr.Ellipsoid("test"), sr.ParameterType.STATE, sr.StateType.GEOMETRY_ELLIPSOID,
               Helpers().assert_state_equal)]


@pytest.mark.parametrize("name,value,parameter_type,state_type,test_func", parameters)
def test_parameter_construction(name, value, parameter_type, state_type, test_func):
    param = sr.Parameter(name, parameter_type, state_type)
    assert param.get_name() == name
    assert param.get_parameter_type() == parameter_type
    assert param.get_parameter_state_type() == state_type
    assert param.is_empty()
    assert not param

    new_param = sr.Parameter(param)
    assert new_param.is_empty()

    param.set_value(value)
    assert param
    assert not param.is_empty()
    test_func(param.get_value(), value)

    param1 = sr.Parameter(name, value, parameter_type, state_type)
    assert param1
    assert not param1.is_empty()
    test_func(param1.get_value(), value)

    new_param = sr.Parameter(param1)
    assert not new_param.is_empty()
    test_func(param1.get_value(), new_param.get_value())

    param.reset()
    assert param.is_empty()


def param_map_equal(param_dict, param_map):
    def simple_param_equal(param1, param2):
        assert param1.get_name() == param2.get_name()
        assert param1.get_type() == param2.get_type()
        assert param1.get_value() == param2.get_value()

    for name, param in param_dict.items():
        p = param_map.get_parameter(name)
        simple_param_equal(p, param)

        value = param_map.get_parameter_value(name)
        assert value == param.get_value()

    for n, p in param_map.get_parameters().items():
        simple_param_equal(p, param_dict[n])

    for p in param_map.get_parameter_list():
        simple_param_equal(p, param_dict[p.get_name()])


def test_param_map():
    param_dict = {"int": sr.Parameter("int", 1, sr.ParameterType.INT),
                  "double": sr.Parameter("double", 2.2, sr.ParameterType.DOUBLE),
                  "string": sr.Parameter("string", "test", sr.ParameterType.STRING)}
    param_list = [param for name, param in param_dict.items()]

    m = sr.ParameterMap()
    for name, param in param_dict.items():
        m.set_parameter(param)
    param_map_equal(param_dict, m)

    m = sr.ParameterMap()
    for name, param in param_dict.items():
        m.set_parameter_value(param.get_name(), param.get_value(), param.get_parameter_type(),
                              param.get_parameter_state_type())
    param_map_equal(param_dict, m)

    m = sr.ParameterMap()
    m.set_parameters(param_dict)
    param_map_equal(param_dict, m)

    m = sr.ParameterMap()
    m.set_parameters(param_list)
    param_map_equal(param_dict, m)

    m = sr.ParameterMap(param_dict)
    param_map_equal(param_dict, m)

    m = sr.ParameterMap(param_list)
    param_map_equal(param_dict, m)

    m.remove_parameter("int")
    with pytest.raises(sr.exceptions.InvalidParameterError):
        m.remove_parameter("int")
    with pytest.raises(sr.exceptions.InvalidParameterError):
        m.get_parameter("int")
