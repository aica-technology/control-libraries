#!/usr/bin/env bash

IMAGE_NAME=aica-technology/control-libraries/control-loop-examples

DOCKER_BUILDKIT=1 docker build "${BUILD_FLAGS[@]}" . -t "${IMAGE_NAME}" || exit 1
docker run -it --rm "${IMAGE_NAME}"