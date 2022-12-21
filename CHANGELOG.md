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
- Remove timestamp setter and rename getter (#2)

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