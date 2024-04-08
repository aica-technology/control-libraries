<!-- include general status checks here -->
<span>
    <img alt="build-release workflow badge" src="https://github.com/aica-technology/control-libraries/actions/workflows/build-release.yml/badge.svg">
    <img alt="build-test workflow badge" src="https://github.com/aica-technology/control-libraries/actions/workflows/build-test.yml/badge.svg?branch=main">
    <img alt="contribution license agreement workflow badge" src="https://github.com/aica-technology/control-libraries/actions/workflows/check-contribution.yml/badge.svg">
    <img alt="page build and deployment workflow badge" src="https://github.com/aica-technology/control-libraries/actions/workflows/pages/pages-build-deployment/badge.svg">
</span>

# Control Libraries

The `control-libraries` project is a collection of modules to facilitate the creation of control loop algorithms for
robotics, including trajectory planning, kinematics, dynamics and control.

Code documentation is available at
[aica-technology.github.io/control-libraries](https://aica-technology.github.io/control-libraries).

The rolling version of the project is available on the
[`main`](https://github.com/aica-technology/control-libraries/tree/main) branch. Refer to the
[Releases](https://github.com/aica-technology/control-libraries/releases) page for other versions.

## Core libraries

The core libraries are implemented in C++ and comprise the following modules:

- `state_representation`
- `dynamical_systems`
- `robot_model`
- `controllers`

Source code, documentation and installation instructions are available under the [source](./source) folder.

## Protocol

There is a module that defines the protocol for sending and receiving messages containing control libraries data across
any network, based on the Google Protocol Buffer. For its implementation, installation and documentation, see the
[protocol](./protocol) folder.

## Python bindings

There exist Python bindings for the control library modules and the protocol module. See the [python](./python)
folder for installation instructions.

## Demos

For examples and demos in C++ and Python, refer to the [demos](./demos) folder.

## Contributing

We welcome user engagement to find bugs, resolve issues and suggest useful features. Refer to the
[contribution guidelines](./CONTRIBUTING.md) for more information.

## License

This project is provided free and open-source under the GPLv3 license. See the [licenses](./licenses) folder for more
information.

## Installation

### Supported platforms

These libraries have been developed and tested on Linux Ubuntu 20.04 and 22.04. They should also work on macOS and
Windows, though the installation steps may differ. At this time no guarantees are made for library support on non-Linux
systems.

### Installation with the install script

This project uses CMake to generate static library objects for each of the modules. To facilitate the installation
process, an [install script](./install.sh) is provided.

The install script takes care of all the installation steps, including the installation and configuration of all
dependencies. It can be run with several optional arguments:
- `-y`, `--auto`: Any input prompts will be suppressed and install steps automatically approved.
- `-d [path]`, `--dir [path]`: If provided, the installation directory will be changed to `[path]`.
- `--clean`: Any previously installed header files from `/usr/local/include` and any shared library files from
  `/usr/local/lib` will be deleted before the installation.
- `--cleandir [path]`: Any previously installed header files shared library files from `[path]` will be deleted before
  the installation.

### Advanced options

Users who prefer to perform the installation manually and/or have already installed some dependencies can selectively
do the steps from the install script.

The CMake configuration flags for control libraries `BUILD_CONTROLLERS`, `BUILD_DYNAMICAL_SYSTEMS` and
`BUILD_ROBOT_MODEL` determine which modules are built, and are all defined as `ON` by default. The building of the
`state_representation` library cannot be disabled, as all other libraries depend on it. To selectively disable the build
of a particular module, set the flag to `=OFF`. For example, the following flags will prevent the `robot_model` module
from being built, which is useful if the Pinocchio dependency is not fulfilled on your system. 

```shell script
-DBUILD_ROBOT_MODEL=OFF
```

Similarly, if one is not interested in the installation of the [protocol](./protocol/README.md), it can be disabled with

```shell script
-DBUILD_PROTOCOL_=OFF
```

The C++ `clproto` library requires control libraries [`state_representation`](../source/state_representation/README.md)
and [Google Protobuf](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md)
to be installed on your computer, which includes the compiler `protoc` and the runtime library `libprotobuf.so`.

To also build the tests, add the CMake flag `-DBUILD_TESTING=ON`. This requires GTest to be installed on your system.
You can then use `make test` to run all test targets.

Alternatively, you can include the source code for each library as submodules in your own CMake project, using the CMake
directive `add_subdirectory(...)` to link it with your project.

### Installation of Python bindings

You must first install the C++ modules before you can install the Python bindings. Additionally, the installation of the
bindings requires the following prerequisites:
- `python3` >= 3.0
- `pip3` >= 10.0.0

The installation itself is then quite straightforward:
```shell
python3 pip install ./python
```

If the installation fails, it may be because of non-default installation directories for some dependencies. In this
case, the include path for OSQP can be set through environment variables before the pip install.

```shell
export OSQP_INCLUDE_DIR='/path/to/include/osqp' # replace /path/to/include with installation directory
python3 pip install ./python
```

The example above installs the module to the default dist-packages location. You can see more information about the
installed module using `pip3 show control-libraries`.

The process also works with Python virtual environments. For example, with `pipenv`:

```shell script
## pip3 install pipenv

pipenv install ./python
```

## External resources

- [Modulo: an extension layer to ROS2 based on control libraries](https://github.com/aica-technology/modulo)
