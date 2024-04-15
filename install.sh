#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" &>/dev/null && pwd)"

INSTALL_DESTINATION="/usr/local"
AUTO_INSTALL=""

PINOCCHIO_TAG=v2.9.0
HPP_FCL_TAG=v1.8.1

FAIL_MESSAGE="The provided input arguments are not valid.
Run the script with the '--help' argument."

HELP_MESSAGE="Usage: [sudo] ./install.sh [OPTIONS]

An install script for the control libraries.

Options:
  -y, --auto               Suppress any input prompts and
                           automatically approve install steps.
  -d, --dir [path]         Configure the installation directory
                           (default: ${INSTALL_DESTINATION}).

  --clean                  Delete any previously installed header
                           files from /usr/local/include and any
                           shared library files from /usr/local/lib.
  --cleandir [path]        Delete any previously installed header
                           and library files from the specified path.

  -h, --help               Show this help message."

function uninstall {
  function delete_components {
    rm -r "${INSTALL_DESTINATION}"/include/controllers
    rm -r "${INSTALL_DESTINATION}"/include/dynamical_systems
    rm -r "${INSTALL_DESTINATION}"/include/robot_model
    rm -r "${INSTALL_DESTINATION}"/include/state_representation
    rm -r "${INSTALL_DESTINATION}"/lib/libcontrollers*.so
    rm -r "${INSTALL_DESTINATION}"/lib/libdynamical_systems*.so
    rm -r "${INSTALL_DESTINATION}"/lib/librobot_model*.so
    rm -r "${INSTALL_DESTINATION}"/lib/libstate_representation*.so
    rm -r "${INSTALL_DESTINATION}"/include/clproto
    rm -r "${INSTALL_DESTINATION}"/lib/libclproto*.so
  }

  delete_components >/dev/null 2>&1

  echo "Deleted any control library artefacts from ${INSTALL_DESTINATION}."
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    -y|--auto) AUTO_INSTALL="-y"; shift 1;;
    --clean) uninstall; exit 0;;
    --cleandir) INSTALL_DESTINATION=$2; uninstall; exit 0;;
    -d|--dir) INSTALL_DESTINATION=$2; shift 2;;
    -h|--help) echo "$HELP_MESSAGE"; exit 0;;
    -*) echo "Unknown option: $1" >&2; echo "$FAIL_MESSAGE"; exit 1;;
  esac
done

mkdir -p "${SCRIPT_DIR}"/tmp || exit 1

echo ">>> INSTALLING DEPENDENCIES"

apt update
xargs -a <(awk '! /^ *(#|$)/' "${SCRIPT_DIR}/apt-packages.txt") -r -- apt install "${AUTO_INSTALL}"

cd "${SCRIPT_DIR}"/tmp
cp "${SCRIPT_DIR}"/dependencies/base_dependencies.cmake CMakeLists.txt || exit 1
cmake -B build -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release && cmake --build build && cmake --install build || exit 1
rm -rf build
git clone --depth 1 -b ${HPP_FCL_TAG} --recursive https://github.com/humanoid-path-planner/hpp-fcl || exit 1
cmake -B build -S hpp-fcl -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF && cmake --build build --target all install || exit 1
rm -rf build
git clone --depth 1 -b ${PINOCCHIO_TAG} --recursive https://github.com/stack-of-tasks/pinocchio
cmake -B build -S pinocchio -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_COLLISION_SUPPORT=ON && cmake --build build --target all install || exit 1
rm -rf build
cp "${SCRIPT_DIR}"/dependencies/dependencies.cmake CMakeLists.txt || exit 1
cmake -B build -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release && cmake --build build && cmake --install build || exit 1
rm -rf build

echo ">>> INSTALLING CONTROL LIBRARIES"
cd "${SCRIPT_DIR}" && rm -rf "${SCRIPT_DIR}"/tmp
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build && cmake --install build --prefix "${INSTALL_DESTINATION}" || exit 1
rm -rf build
