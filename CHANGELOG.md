# CHANGELOG

Release Versions

- [9.1.0](#910)
- [9.0.1](#901)
- [9.0.0](#900)
- [8.1.0](#810)
- [8.0.0](#800)
- [7.4.0](#740)
- [7.3.0](#730)
- [7.2.0](#720)
- [7.1.1](#711)
- [7.1.0](#710)
- [7.0.0](#700)
- [6.3.1](#631)
- [6.3.0](#630)
- [6.2.0](#620)

## Upcoming changes

- feat: improve support for transformation matrices (#146)
- fix: unstable python test (#221)
- feat: improve devcontainer configuration (#230)
- chore: format code (#235)
- ci: allow workflow dispatch for rc images (#234)
- feat: move up to C++20 (#220)
- feat: split trajectory classes and implement Cartesian/JointState specializations (#216, #217, #223)
- feat: add trajectory class bindings and python tests (#219)

## 9.1.0

Version 9.1.0 adds `integrate` and `differentiate` methods to relevant Cartesian and joint states to allow integration
and differentiation with double types as well as `chrono` types. Additionally, this version improves the performance of
the Inverse Kinematics algorithm in the `robot_model` library.

### Full changelog

- feat: add integrate and differentiate methods to Cartesian and joint types (#193)
- fix(robot-model): improve ik performance (#205)

## 9.0.1

Version 9.0.1 is a patch version update that exposes Cartesian and Joint state utility functions from the 
corresponding `state_representation` classes.

### Features

It is now possible to convert `CartesianStateVariable` and `JointStateVariable`s from and to `std::string`.

### Full changelog

- feat(state-representation): add utilities for CartesianStateVariable (#195, #201)
- feat(state-representation): add utilities for JointStateVariable (#197, #201)
- feat(robot-model): add clamp_in_range function for individual JointStateVariable (#194)
  
## 9.0.0

Version 9.0.0 is a new major version of control libraries that is built on Ubuntu 24.04 with Python 3.12. It does not
contain any new features or fixes compared to version 8.1.0.

## 8.1.0

Version 8.1.0 adds a new module called `communication_interfaces` to the control libraries. This is a library for
simple socket communication and was previously developed in a different place. It currently implements sockets for UPD,
TCP, and ZMQ communication.

### Full changelog

- feat: migrate communication interfaces (#190)

## 8.0.0

Version 8.0.0 is a major update that adds new state types and collision detection features to control-libraries.

### Breaking changes

**robot_model**

As part of the collision detection features in `robot_model::Model`, the dependencies of the `robot_model` module have
changed which constitutes a breaking change that requires users of the module to rebuild their code.

**General**

As of version 8.0.0, the `develop` branch will be deleted and the Linear Git-Flow workflow will be abandoned in favor of
easier release cycles (see [CONTRIBUING](./CONTRIBUTING.md)). Additionally, the `development-dependencies` and
`proto-dependencies` images will no longer be supported or maintained.

### Features

The `robot_model` module now supports collision detection features from pinocchio, allowing to retrieve minimum
distances between links.

### General

The behind-the-scenes structural improvements to the build system and the CI that have existed in parallel since version
7.1.0 have now replaced the legacy build system with multiple shell scripts and Dockerfiles. All information regarding
building and using control-libraries should still be available in the READMEs.

### Full changelog

- feat: add metadata to docker image (#188)
- chore: touch up workflows and documentation (#181)
- feat: update demos directory (#179)
- fix: update copy constructor to avoid warnings (#180)
- build: remove deprecated Dockerfiles and scripts and update installation instructions (#176)
- refactor: optimize copy and swap constructor for robot model (#174)
- fix: refactor cmake project to deal with robot model dependencies (#178)
- feat: integrate minimum distance calculation feature into robot model(#167)
- ci: update workflows (#175)
- feat: integrate collision detection feature into robot model (#163)
- ci: use caching from docker to run tests in CI (#169)
- build: add missing licenses (#170)
- feat(build): handle installation and linking of dependencies for pinocchio collision support (#161)

## 7.4.0

Version 7.4.0 brings Analog and Digital IO State types as a new feature to the `state_representation` module.

### Features

`AnalogIOState` and `DigitalIOState` classes have been added as new state types in `state_representation`.

### Full changelog

- feat: add IO states to state representation (py) (#173)
- feat: add IO states to state representation (proto) (#172)
- feat: add IO states to state representation (cpp) (#158)
- build: update dockerfiles (#153)
- build: copy python packages into /usr instead of ~ros2 to avoid permission issues (#155)
- feat: Add ParameterType conversion functions to go from enum to type label and the inverse (#154)

## 7.3.0

Version 7.3.0 contains new improvements and a fix to the control libraries.

### Features

Setting controller gain parameters is now even easier than before as vectors and arrays of size 1 are also allowed and
interpreted the same way as a double. Additionally, the robot model now has an improved inverse velocity calculation
that uses a damped least squared pseudoinverse if desired.

### Fix

An error in the scalar multiplication operator of a Cartesian state that generated an incorrect orientation has now been
fixed.

### Full changelog

- feat(robot model): Damped least squared pseudoinverse (#143)
- feat(controllers): improve parameter validation of impedance controller (#148)
- fix(state_representation): remove error in orientation scaling (#147)

## 7.2.0

Version 7.2.0 contains improvements for the Python bindings of control libraries.

### Features

To enable a good development experience with IDEs that cannot introspect pybind modules, so-called stubs are now
generated for each Python module and shipped alongside the actual modules.

### Fixes

Since version 7.1.0, some of the Python modules have not been built in the Docker image due to missing or wrong
dependencies. This has been fixed in #134.

### Full changelog:

- feat(build): add configuration files for VS Code devcontainer (#137, #138)
- feat(python): auto-generate stubs for python modules in Dockerfile (#135, #139)
- fix: build all python modules (#134)

## 7.1.1

Version 7.1.1 contains one commit to fix permissions of the home folder in the generated Docker images.

## 7.1.0

Version 7.1.0 contains behind-the-scenes structural improvements to the build system and the CI as well as a few
minor improvements and fixes to state_representation.

### Full changelog:

- Update push hooks in build-release.yaml (#127)
- feat(ci): add prebuilt control-libraries image akin to network-interfaces (#125)
- Add overloaded `make_shared_parameter` with no parameter (#123)
- Catch out of range exception in `ParameterMap::assert_parameter_valid` (#122)
- Construct zero matrices in compliant twist controller (#120)

## 7.0.0

Version 7.0.0 is a major update to the `state_representation` module to make `State` types more internally consistent
and safe to use.

### Breaking changes

**state_representation**

The `State` class has been reworked, with the following breaking changes to the API:

- `State::initialize` has been reworked as `State::reset`.
- `State::is_compatible` has been removed in favor of the new `State::is_incompatible`.
- `State::set_empty` and `State::set_filled` have been removed from the public API.

In addition, the `StateType` of a state can no longer be changed during or after construction, and is determined
uniquely by the class implementation.

The `CartesianState` class and derived classes have revised operators for addition, subtraction and product operators
to promote consistency and safety. Invalid operations are now explicitly prohibited to prevent undesirable implicit type
casting, and the return type of these operations now follows a standard pattern.

The `JointState` class and derived classes had similar changes.

The `Jacobian` class is safer to use; `Jacobian::transpose`, `Jacobian::inverse` and `Jacobian::pseudoinverse` now
return a matrix in the default case instead of a modified `Jacobian` object and have overloaded variants for direct
calculations with other types. This means a `Jacobian` instance always represents the normal, non-transposed or
non-inverted case. `Jacobian::set_reference_frame` now takes a string instead of a `CartesianPose`.

**General**

All header files now use the `.hpp` extension.

### Features and improvements

**state_representation**

The timestamp of a `State` object is now consistently reset by every non-const method, so that it always indicates the
time of last modification. The method `State::get_age()` can be used to get the time since last modification in seconds.

The emptiness of a state is handled more consistently, and accessing data on an empty state will now throw an exception.

Quaternion manipulation, conjugation and differentiation is handled in a more internally consistent way in regard to
the positive or negative vector representation. The random orientation of a `CartesianPose` now correctly samples a
uniform distribution of the orientation quaternion space instead of a unit 4-vector space.

Manipulation of `CartesianState` wrench is now more consistent and explicit, in particular for transform and inverse.

**controllers**

The controllers now have a `forward_force` parameters that can toggle whether the `CartesianWrench` or `JointTorque`
of a feedback state is passed through to the command as a feed-forward force or not.

**Python**

The Python bindings now include the exceptions for each module. For example:

- `from state_representation.exceptions import EmptyStateError`

The bindings have also been updated to include the breaking changes in `state_representation`.

**clproto**

The message protocol has been simplified and no longer encodes the timestamp or state type, since both of these
properties are set on construction. The data of an empty state is also ignored when it is encoded or decoded, allowing
for more efficient and consistent serialized messages.

### Documentation

The `state_representation` module
[documentation](https://github.com/aica-technology/control-libraries/tree/main/source/state_representation)
has been rewritten for more clarity and completeness. Additionally, the main repository README, demo folder and
contribution guidelines have been updated.

### Behind the scenes

The build system has been improved and uses pkg-config files to resolve dependencies during installation.

A contributor license agreement and signature workflow have been added to protect and encourage open source development.

### Full changelog:

- Fix angular stiffness calculation in impedance controller (#117)
- Update documentation for contribution and demos (#113)
- Implement frame name setter for Jacobian (#111)
- Update state representation documentation (#107, #110)
- Consistently reset timestamp if state data has been changed (#108)
- Update python bindings after Jacobian refactor (#105)
- Set reference frame of Jacobian with string instead of pose (#103)
- Refactor Jacobian methods and operators (#101)
- Remove rows* and cols* and improve constructors (#100)
- Revise clproto to remove StateType and timestamp from State message (#104)
- Raise exception in getters if state is empty (#97)
- Bind error objects of all library modules with Pybind (#98)
- Move inlined functions of geometry classes to source files (#95)
- Setting state to zero results in non-empty state (#93)
- Clean up geometry classes to define emptiness (#91)
- Fix wrench product and inverse (#53)
- Require CLA signatures with PR workflow (#89)
- Use hpp extension for header files consistently (#87)
- Clean up parameter container class and tests (#85)
- Use organization build and push workflow (#82)
- Bind state representation error objects with Pybind (#83)
- Fix encoding of empty parameters (#78)
- Automatic update of version and changelog upon PR creation (#81)
- Revert order of upcoming changes in CHANGELOG (#80)
- Refactor clproto tests (#75)
- Correct parameter map in impedance controller (#76)
- Rename State::initialize to State::reset (#73)
- Use pytest to run Python bindings tests (#70)
- Ignore data fields when copying an empty state (#71)
- Allow Parameter construction with empty name (#67)
- Add initialize method to Parameter class (#68)
- Bind operators of joint states (#65)
- Refactor operators in JointState for multiplication & division (#62), addition (#63) and subtraction (#64)
- Bind operators of Cartesian states (#60)
- Fix quaternion differentiation (#58)
- Refactor operators in CartesianState for addition (#23), subtraction (#33, #40), transformation (#28, #45),
  division (#41) and multiplication (#43)
- Add a Contributor License Agreement (#61)
- Uniformly sample orientation with UnitRandom (#56)
- Refactor get/set variable helpers in Cartesian and joint state (#39, #57)
- Bind inverse for each Cartesian class individually (#52)
- Simplify and consolidate ostream operators (#37)
- Add forward force parameter to Impedance controller (#36)
- Remove state type from clproto (#47)
- Set state type in copy constructors (#34)
- Fix inverse operation in CartesianState (#32)
- Move helpers to source file (#21)
- Remove StateType argument from State constructor (#16)
- Set the State::set_empty method protected (#14)
- No encoding and decoding of data fields in empty states (#9)
- Refactor State::is_compatible to is_incompatible (#6)
- Remove empty flag from State and SpatialState constructors (#7)
- Reorganize function declarations and implementations (#4, #8)
- Remove type from SpatialState constructor (#3)
- Get the age of a state and remove timestamp setter (#2)
- Transfer repository ownership to AICA (#1)
- Use pkg-config to check for dependencies (epfl-lasa#328)
- Add pkg-config files for clproto (epfl-lasa#327)
- Add pkg-config files for control libraries (epfl-lasa#326)

## 6.3.1

Version 6.3.1 is a patch release to fix the modified installation instructions for the Eigen library.

### Fixes

- Fix Eigen installation flags (epfl-lasa#323)

## 6.3.0

Version 6.3.0 contains behind-the-scenes structural improvements to Dockerfiles and GitHub workflows
and clarifies the license requirements of the library and its dependencies.

### Fixes and improvements

- Build and push both 20.04 and 22.04 images (epfl-lasa#314, epfl-lasa#315, epfl-lasa#316)
- Don't build pinocchio tests in development image (epfl-lasa#317)
- Simplify and consolidate Dockerfiles and scripts (epfl-lasa#319)
- Better license management (epfl-lasa#320)

## 6.2.0

Version 6.2.0 is the most recent release of control-libraries authored by AICA employees under EPFL employement
and granted to AICA under a non-exclusive license agreement. All subsequent contributions have been made solely by
AICA employees and are the property of AICA SA. For a full history of changes prior to version 6.2, refer to
the original project here:

- https://github.com/epfl-lasa/control-libraries/tree/v6.2.0
