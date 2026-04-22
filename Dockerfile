ARG BASE_TAG=24.04
ARG PINOCCHIO_TAG=v0.1.0
FROM ghcr.io/aica-technology/pinocchio:${PINOCCHIO_TAG} AS pinocchio

FROM ubuntu:${BASE_TAG} AS base
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    git \
    libgtest-dev \
    python3-pip \
    ssh \
    sudo \ 
    clangd \
    clang-format
RUN apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN echo "Set disable_coredump false" >> /etc/sudo.conf

# create the credentials to be able to pull private repos using ssh
RUN mkdir -p /root/.ssh/ && ssh-keyscan github.com | tee -a /root/.ssh/known_hosts

ARG CMAKE_BUILD_TYPE=Release

FROM base AS apt-dependencies
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

FROM base AS dependencies
ARG TARGETPLATFORM
ARG CACHEID
ARG OSQP_TAG=v0.6.3
COPY --from=apt-dependencies /tmp/apt /

RUN --mount=type=cache,target=/build,id=cmake-osqp-${OSQP_TAG}-${TARGETPLATFORM}-${CACHEID},uid=1000 \
<<EOF
set -e

if [ ! -f osqp/CMakeLists.txt ]; then
  rm -rf osqp/*
  git clone --depth 1 -b ${OSQP_TAG} --recursive https://github.com/oxfordcontrol/osqp
fi

cmake -B build/osqp -S osqp -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_INSTALL_PREFIX=/tmp/deps
cmake --build build/osqp --target all install
EOF

COPY dependencies/dependencies.cmake CMakeLists.txt
RUN --mount=type=cache,target=/build,id=cmake-deps-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  cmake -B build -Dprotobuf_BUILD_TESTS=OFF -DCPPZMQ_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} -DCMAKE_PREFIX_PATH=/tmp/deps \
  && cmake --build build && cmake --install build --prefix /tmp/deps

FROM base AS code
COPY --from=apt-dependencies /tmp/apt /
COPY --from=dependencies /tmp/deps /usr
COPY --from=pinocchio / /
ENV LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu/openblas-pthread:$LD_LIBRARY_PATH
ENV PYTHONPATH=/usr/lib/python3.12/site-packages:$PYTHONPATH

FROM code AS development
ARG USER=ubuntu

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

# Configure sshd entrypoint to authorize the new user for ssh access and
# optionally update UID and GID when invoking the container with the entrypoint script
COPY ./docker/sshd_entrypoint.sh /sshd_entrypoint.sh
RUN chmod 744 /sshd_entrypoint.sh

RUN mkdir /guidelines && cd /guidelines \
  && wget https://raw.githubusercontent.com/aica-technology/.github/v1.0.4/guidelines/.clang-format

USER ${USER}
WORKDIR /src
COPY --chown=${USER}:${USER} . .

FROM code AS build
ARG TARGETPLATFORM
ARG CACHEID
COPY licenses licenses
COPY protocol protocol
COPY source source
COPY CMakeLists.txt CMakeLists.txt
RUN --mount=type=cache,target=/build,id=cmake-build-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  cmake -B build -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} && cmake --build build

FROM build AS cpp-test
ARG TARGETPLATFORM
ARG CACHEID
RUN --mount=type=cache,target=/build,id=cmake-build-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  cmake -B build -DBUILD_TESTING=ON && make -C build && CTEST_OUTPUT_ON_FAILURE=1 make -C build test

FROM build AS install
ARG TARGETPLATFORM
ARG CACHEID
RUN --mount=type=cache,target=/build,id=cmake-build-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  cmake --install build --prefix /tmp/cl

FROM code AS python
ARG TARGETPLATFORM
ARG CACHEID
COPY --from=install /tmp/cl /usr
COPY ./python/include /python/include
COPY ./python/source /python/source
COPY ./python/pyproject.toml ./python/setup.py /python/
RUN --mount=type=cache,target=/.cache,id=pip-${TARGETPLATFORM}-${CACHEID},uid=1000 \
  python3 -m pip install --prefix=/tmp/python /python
RUN mv /tmp/python/local /tmp/python-usr

FROM cpp-test AS python-test
RUN pip install pytest --break-system-packages
COPY --from=install /tmp/cl /usr
COPY --from=python /tmp/python-usr /usr
COPY ./python/test /test
RUN pytest /test

FROM code AS python-stubs
ARG TARGETPLATFORM
ARG CACHEID
COPY --from=install /tmp/cl /usr
COPY --from=python /tmp/python-usr /usr
RUN pip install pybind11-stubgen --break-system-packages
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

FROM scratch AS production
COPY --from=apt-dependencies /tmp/apt /
COPY --from=dependencies /tmp/deps /usr
COPY --from=install /tmp/cl /usr
COPY --from=python /tmp/python-usr /usr
COPY --from=python-stubs /tmp/python-usr /usr

ARG VERSION
LABEL org.opencontainers.image.title="AICA control-libraries"
LABEL org.opencontainers.image.description="AICA control libraries"
LABEL org.opencontainers.image.version="${VERSION}"
LABEL tech.aica.image.metadata='{"type":"lib"}'
