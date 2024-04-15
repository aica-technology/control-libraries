# Core Libraries

## `state_representation`

This library provides a set of classes to represent **states** in **Cartesian** or **joint** space.
The classes define and combine variables such as position, velocity, acceleration and force into
a consistent internal representation used across the other libraries.

Source: [state_representation](./state_representation)

Dependencies: [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page)

---

## `dynamical_systems`

This library provides a collection of classes that behave as differential equations to calculate
a derivative of a state variable. For example, a position state input might
yield a desired velocity output. This can be used to generate time-invariant trajectories.

Source: [dynamical_systems](./dynamical_systems)

Dependencies: `state_representation`

---

## `robot_model`

This library allows the creation of a robot model from a URDF file and defines many helpful
rigid-body algorithms for solving kinematic and dynamic problems.

It is a wrapper for [Pinocchio](https://github.com/stack-of-tasks/pinocchio)
that is compatible with the internal `state_representation` types.

Source: [robot_model](./robot_model)

Dependencies: `state_representation`, [Pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)

---

## `controllers`

This library provides classes designed to convert an input state to an output command in order to control
a robot.

Source: [controllers](./controllers)

Dependencies: `state_representation`, `robot_model`

---

## Usage in a cmake project

If you have a target library or executable `my_target`, you can link all required libraries
and include directories automatically with the following commands in the CMakeLists file:

```cmake
find_package(control_libraries [VERSION] CONFIG REQUIRED)

target_link_libraries(my_target ${control_libraries_LIBRARIES})
```

`VERSION` is an optional version number such as `5.0.0`, which should be included for safety. If the installed
control libraries version has a different major increment (for example, `4.1.0` compared to `5.0.0`), cmake
will give the according error.

The `CONFIG` flag tells cmake to directly search for a `control_librariesConfig.cmake` file, which is installed
as part of the package. It is not required but makes the cmake search algorithm slightly more efficient.

The `REQUIRED` flag will give an error if the package is not found, which just prevents build errors from happening
at later stages.

### Advanced specification

For more fine control in checking required components when importing control libraries, it is possible to
supply the individual required libraries to the end of the `find_package` directive. The default behaviour
is to include all installed components.

If all components are installed but only the `state_representation` library is needed for a certain project,
then it is a good idea to only find that component. Note however that for components specified at the find_package step,
the linked libraries must also be specified from that subset, and not from the `${control_libraries_LIBRARIES}`variable.

```cmake
# only find dependencies for the state_representation library
find_package(control_libraries 5.0.0 CONFIG REQUIRED state_representation)

target_link_libraries(my_target state_representation)
```

The other usage for listing COMPONENTS during `find_package` is to confirm that those components are indeed installed,
and to abort the cmake configuration early if the required component was not found (for example, when the
control libraries installation selectively disabled the `controllers` library).

Additional `OPTIONAL_COMPONENTS` can also be appended, which will silently check if they are installed without aborting.
This is generally not needed.

```cmake
# ensure that both `dynamical_systems` and `robot_model` are installed and available
find_package(control_libraries 5.0.0 CONFIG REQUIRED dynamical_systems robot_model)

# ensure that the `controllers` library is available, and also check for dynamical_systems in the background
find_package(control_libraries 5.0.0 CONFIG REQUIRED controllers OPTIONAL_COMPONENTS dynamical_systems)
```
