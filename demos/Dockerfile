FROM ghcr.io/aica-technology/control-libraries:rolling as cl
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    cmake \
    g++ \
    python3 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

COPY --from=cl / /

WORKDIR /control_loop_examples
COPY ./ ./

RUN mkdir build && cd build && cmake .. && make -j all && make install
