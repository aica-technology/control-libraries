ARG BASE_TAG=22.04
FROM ubuntu:${BASE_TAG} as base
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    git \
    libgtest-dev \
    libeigen3-dev \
    python3-pip \
    ssh \
    sudo \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN echo "Set disable_coredump false" >> /etc/sudo.conf

# create the credentials to be able to pull private repos using ssh
RUN mkdir /root/.ssh/ && ssh-keyscan github.com | tee -a /root/.ssh/known_hosts

FROM base as apt-dependencies
COPY apt-packages.tx[t] /

RUN <<HEREDOC
if [ ! -s /apt-packages.txt ]; then
  set +e # FIXME: without this, the script fails because of an issue with `clear_console`
  exit 0
fi

mkdir -p /tmp/apt

apt-get update
# We then do a dry-run and parse the output of apt to gather the list of packages to be installed
# Example output:
# ```
# #########
# NOTE: This is only a simulation!
#       apt-get needs root privileges for real execution.
#       Keep also in mind that locking is deactivated,
#       so don't depend on the relevance to the real current situation!
# Reading package lists...
# Building dependency tree...
# Reading state information...
# The following additional packages will be installed:
#   libavutil56 libblosc1
# The following NEW packages will be installed:
#   libavutil56 libblosc1
# 0 upgraded, 5 newly installed, 0 to remove and 28 not upgraded.
# Inst libavutil56 (7:4.4.2-0ubuntu0.22.04.1 Ubuntu:22.04/jammy-updates, Ubuntu:22.04/jammy-security [arm64])
# Inst libblosc1 (1.21.1+ds2-2 Ubuntu:22.04/jammy [arm64])
# Conf libavutil56 (7:4.4.2-0ubuntu0.22.04.1 Ubuntu:22.04/jammy-updates, Ubuntu:22.04/jammy-security [arm64])
# Conf libblosc1 (1.21.1+ds2-2 Ubuntu:22.04/jammy [arm64])
# ```
# Transformed into:
# ```
# libavutil56
# libblosc1
# ```
xargs -a /apt-packages.txt apt-get install --dry-run \
  | grep -e '^Inst ' \
  | sed -E 's/^Inst (\S+) .*$/\1/' > /tmp/new-packages.txt
# Then we install apt packages like normal
xargs -a /apt-packages.txt apt-get install -y
# Finally we use dpkg to get all files installed by those packages and copy them to a new root
#  - get list of files installed by all the packages
#  - remove empty lines
#  - sort
#  - remove duplicates
#  - copy files while keeping file hierarchy and preserving links as-is
#  - remove "omitting directory" messages (we don't do recursive copy as we only want specific files) for cleaner output
xargs -a /tmp/new-packages.txt dpkg-query -L \
  | sed '/^$/d' | sort | uniq \
  | xargs -d "\n" cp --parents -dp -t /tmp/apt  2>&1 \
  | grep -v 'omitting directory'
# this root can then be copied to / to install everything globally or use LD_LIBRARY_PATH to use it locally
HEREDOC

FROM base as base-dependencies
ARG TARGETPLATFORM
ARG CACHEID
COPY dependencies/base_dependencies.cmake CMakeLists.txt
RUN --mount=type=cache,target=/build,id=cmake-base-deps-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build && cmake --install build --prefix /tmp/deps

FROM base as pinocchio-dependencies
COPY --from=apt-dependencies /tmp/apt /
COPY --from=base-dependencies /tmp/deps /usr
ARG TARGETPLATFORM
ARG CACHEID
ARG PINOCCHIO_TAG=v2.9.0
ARG HPP_FCL_TAG=v1.8.1
# FIXME: it would be nicer to have it all in the root CMakelists.txt but:
#  * `pinocchio` doesn't provide an include directory we can easily plug into `target_include_directories` and thus needs to be installed first
#  * `pinocchio` uses hacks relying on undocumented CMake quirks which break if you use `FetchContent`
# FIXME: it needs `CMAKE_INSTALL_PREFIX` and `--prefix` because it doesn't install to the right place otherwise
RUN --mount=type=cache,target=/hpp-fcl,id=cmake-hpp-fcl-src-${HPP_FCL_TAG}-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  --mount=type=cache,target=/build,id=cmake-hpp-fcl-${HPP_FCL_TAG}-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  if [ ! -f hpp-fcl/CMakeLists.txt ]; then rm -rf hpp-fcl/* && git clone --depth 1 -b ${HPP_FCL_TAG} --recursive https://github.com/humanoid-path-planner/hpp-fcl; fi \
  && cmake -B build -S hpp-fcl -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF -DCMAKE_INSTALL_PREFIX=/tmp/deps \
  && cmake --build build --target all install
RUN --mount=type=cache,target=/pinocchio,id=cmake-pinocchio-src-${PINOCCHIO_TAG}-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  --mount=type=cache,target=/build,id=cmake-pinocchio-${PINOCCHIO_TAG}-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  if [ ! -f pinocchio/CMakeLists.txt ]; then rm -rf pinocchio/* && git clone --depth 1 -b ${PINOCCHIO_TAG} --recursive https://github.com/stack-of-tasks/pinocchio; fi \
  && cmake -B build -S pinocchio -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_WITH_COLLISION_SUPPORT=ON -DCMAKE_INSTALL_PREFIX=/tmp/deps \
  && cmake --build build --target all install
# FIXME: pinocchio produces non-portable paths
RUN find /tmp/deps -type f -exec sed -i 's#/tmp/deps#/usr#g' '{}' \;

FROM base as dependencies
ARG TARGETPLATFORM
ARG CACHEID
# Needed to build `osqp-eigen`
COPY --from=base-dependencies /tmp/deps /usr
COPY dependencies/dependencies.cmake CMakeLists.txt
RUN --mount=type=cache,target=/build,id=cmake-deps-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  cmake -B build -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release && cmake --build build && cmake --install build --prefix /tmp/deps
COPY --from=base-dependencies /tmp/deps /tmp/deps
COPY --from=pinocchio-dependencies /tmp/deps /tmp/deps

FROM base as code
COPY --from=apt-dependencies /tmp/apt /
COPY --from=dependencies /tmp/deps /usr

FROM code as development
# create and configure a new user
ARG UID=1000
ARG GID=1000
ENV USER developer
ENV HOME /home/${USER}

RUN addgroup --gid ${GID} ${USER}
RUN adduser --gecos "Remote User" --uid ${UID} --gid ${GID} ${USER} && yes | passwd ${USER}
RUN usermod -a -G dialout ${USER}
RUN echo "${USER} ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/99_aptget
RUN chmod 0440 /etc/sudoers.d/99_aptget && chown root:root /etc/sudoers.d/99_aptget

# Configure sshd server settings
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PubkeyAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_development \
  && mkdir /run/sshd

# Configure sshd entrypoint to authorise the new user for ssh access and
# optionally update UID and GID when invoking the container with the entrypoint script
COPY ./docker/sshd_entrypoint.sh /sshd_entrypoint.sh
RUN chmod 744 /sshd_entrypoint.sh

RUN mkdir /guidelines && cd /guidelines \
  && wget https://raw.githubusercontent.com/aica-technology/.github/v0.9.0/guidelines/.clang-format

USER ${USER}
WORKDIR /src
COPY --chown=${USER}:${USER} . .

FROM code as build
ARG TARGETPLATFORM
ARG CACHEID
COPY licenses licenses
COPY protocol protocol
COPY source source
COPY CMakeLists.txt CMakeLists.txt
RUN --mount=type=cache,target=/build,id=cmake-build-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build

FROM build as cpp-test
ARG TARGETPLATFORM
ARG CACHEID
RUN --mount=type=cache,target=/build,id=cmake-build-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  cmake -B build -DBUILD_TESTING=ON && cd build && make && CTEST_OUTPUT_ON_FAILURE=1 make test

FROM build as install
ARG TARGETPLATFORM
ARG CACHEID
RUN --mount=type=cache,target=/build,id=cmake-build-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  cmake --install build --prefix /tmp/cl

FROM code as python
ARG TARGETPLATFORM
ARG CACHEID
COPY --from=install /tmp/cl /usr
COPY ./python/include /python/include
COPY ./python/source /python/source
COPY ./python/pyproject.toml ./python/setup.py /python/
RUN --mount=type=cache,target=/.cache,id=pip-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  python3 -m pip install --prefix=/tmp/python /python
RUN mv /tmp/python/local /tmp/python-usr

FROM cpp-test as python-test
RUN pip install pytest
COPY --from=install /tmp/cl /usr
COPY --from=python /tmp/python-usr /usr
COPY ./python/test /test
RUN pytest /test

FROM code as python-stubs
ARG TARGETPLATFORM
ARG CACHEID
COPY --from=install /tmp/cl /usr
COPY --from=python /tmp/python-usr /usr
RUN pip install pybind11-stubgen
RUN --mount=type=cache,target=/.cache,id=pip-${TARGETPLATFORM}-${CACHEID},uid=1000 \
<<HEREDOC
for PKG in state_representation dynamical_systems robot_model controllers clproto; do
  python3 -c "import ${PKG}"
  if [ $? -eq 0 ]; then
    pybind11-stubgen $PKG -o ./stubs
    mkdir -p ./stubs-package/$PKG-stubs && find ./stubs -type f -name "*.pyi" -exec mv {} ./stubs-package/$PKG-stubs \;
    cat << EoF > ./stubs-package/setup.py
from distutils.core import setup

import $PKG


setup(
    name="$PKG-stubs",
    author="Dominic Reber",
    author_email="dominic@aica.tech",
    version=$PKG.__version__,
    package_data={"$PKG-stubs": ["*.pyi"]},
    packages=["$PKG-stubs"]
)
EoF
    python3 -m pip install --prefix=/tmp/python ./stubs-package || exit 1
    rm -r ./stubs*
  fi
done
HEREDOC
RUN mv /tmp/python/local /tmp/python-usr

FROM scratch as production
COPY --from=apt-dependencies /tmp/apt /
COPY --from=dependencies /tmp/deps /usr
COPY --from=install /tmp/cl /usr
COPY --from=python /tmp/python-usr /usr
COPY --from=python-stubs /tmp/python-usr /usr
