<!-- include general status checks here -->
<span>
    <img alt="build-push workflow badge" src="https://github.com/aica-technology/control-libraries/actions/workflows/build-push.yml/badge.svg">
    <img alt="contribution license agreement workflow badge" src="https://github.com/aica-technology/control-libraries/actions/workflows/check-contribution.yml/badge.svg">
    <img alt="page build and deployment workflow badge" src="https://github.com/aica-technology/control-libraries/actions/workflows/pages/pages-build-deployment/badge.svg">
</span>

# Control Libraries

The `control-libraries` project is a collection of modules to facilitate the creation of control loop algorithms for
robotics, including trajectory planning, kinematics, dynamics and control.

Code documentation is available at <a href="https://aica-technology.github.io/control-libraries">
aica-technology.github.io/control-libraries</a>.

## Releases

The latest stable version of the project is available on the
[`main`](https://github.com/aica-technology/control-libraries/tree/main), while the latest pre-release development
build is available on the [`develop`](https://github.com/aica-technology/control-libraries/tree/develop) branch.

Refer to the [Releases](https://github.com/aica-technology/control-libraries/releases) page for other versions.

<!-- include branch-specific status checks here -->
<table>
    <tr>
        <td>Branch</td>
        <td>Status</td>
    </tr>
    <tr>
        <td>
            <a href="https://github.com/aica-technology/control-libraries/tree/main">
                <code>main</code>
            </a>
        </td>
        <td>
            <img alt="main branch build-test workflow badge" src="https://github.com/aica-technology/control-libraries/actions/workflows/build-test.yml/badge.svg?branch=main">
        </td>
    </tr>
    <tr>
        <td>
            <a href="https://github.com/aica-technology/control-libraries/tree/develop">
                <code>develop</code>
            </a>
        </td>
        <td>
            <img alt="develop branch build-test workflow badge" src="https://github.com/aica-technology/control-libraries/actions/workflows/build-test.yml/badge.svg?branch=develop">
        </td>
    </tr>
</table>

## Core libraries

The core libraries are implemented in C++ and comprise the following modules:

- `state_representation`
- `dynamical_systems`
- `robot_model`
- `controllers`

Source code, documentation and installation instructions are available under the [source](./source) folder.

## Python bindings

There exist Python bindings for the control library modules and the protocol module. See the [python](./python)
folder for installation instructions.

## Protocol

There is a module that defines the protocol for sending and receiving messages containing control libraries
data across any network, based on the Google Protocol Buffer. For its implementation, installation and
documentation, see the [protocol](./protocol) folder.

## Demos

For examples and demos in C++ and Python, refer to the [demos](./demos) folder.

## Contributing

We welcome user engagement to find bugs, resolve issues and suggest useful features.
Refer to the [contribution guidelines](./CONTRIBUTING.md) for more information.

## License

This project is provided free and open-source under the GPLv3 license.
See the [licenses](./licenses) folder for more information.

## External resources

- [Docker images with control libraries installations](https://github.com/aica-technology/docker-images)
- [Modulo: an extension layer to ROS2 based on control libraries](https://github.com/aica-technology/modulo)
