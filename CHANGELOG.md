# CHANGELOG

Release Versions:
- [6.3.1](#631)
- [6.3.0](#630)
- [6.2.0](#620)

## Upcoming changes (in development)

- Add pkg-config files for control libraries (epfl-lasa#326)
- Add pkg-config files for clproto (epfl-lasa#327)
- Use pkg-config to check for dependencies (epfl-lasa#328)
- Transfer repository ownership to AICA (#1)
- Get the age of a state and remove timestamp setter (#2)
- Remove type from SpatialState constructor (#3)
- Reorganize function declarations and implementations (#4, #8)
- Remove empty flag from State and SpatialState constructors (#7)
- Refactor State::is_compatible to is_incompatible (#6)
- No encoding and decoding of data fields in empty states (#9)
- Set the State::set_empty method protected (#14)
- Remove StateType argument from State constructor (#16)
- Move helpers to source file (#21)
- Fix inverse operation in CartesianState (#32)
- Set state type in copy constructors (#34)
- Remove state type from clproto (#47)
- Add forward force parameter to Impedance controller (#36)
- Simplify and consolidate ostream operators (#37)
- Bind inverse for each Cartesian class individually (#52)
- Refactor get/set variable helpers in Cartesian and joint state (#39, #57)
- Uniformly sample orientation with UnitRandom (#56)
- Add a Contributor License Agreement (#61)
- Refactor operators in CartesianState (addition #23, subtraction #33 and #40, 
  transformation #28 and #45, division #41, multiplication #43)
- Fix quaternion differentiation (#58)
- Bind operators of Cartesian states (#60)
- Refactor operators in JointState (multiplication & division #62, addition #63, subtraction #64)
- Bind operators of joint states (#65)
- Add initialize method to Parameter class (#68)
- Allow Parameter construction with empty name (#67)
- Ignore data fields when copying an empty state (#71)
- Use pytest to run Python bindings tests (#70)
- Rename State::initialize to State::reset (#73)
- Correct parameter map in impedance controller (#76)
- Refactor clproto tests (#75)

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