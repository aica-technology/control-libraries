import pytest

import state_representation as sr


def assert_list_equal(value, expected_value):
    assert len(value) == len(expected_value)
    assert all([a == b for a, b in zip(value, expected_value)])


io_states = [(sr.DigitalIOState, "io", ["1", "2"], [True, False], sr.StateType.DIGITAL_IO_STATE),
             (sr.AnalogIOState, "io", ["1", "2"], [0.5, 1.1], sr.StateType.ANALOG_IO_STATE)]


@pytest.mark.parametrize("class_type,name,io_names,values,state_type", io_states)
def test_construction(class_type, name, io_names, values, state_type):
    state = class_type(name, io_names)
    assert state.get_name() == name
    assert state.get_type() == state_type
    assert state.is_empty()
    assert not state

    new_state = class_type(state)
    assert new_state.get_type() == state_type
    assert new_state.is_empty()

    state.set_data(values)
    assert state
    assert not state.is_empty()
    assert_list_equal(state.to_list(), values)

    state.reset()
    assert state.is_empty()
