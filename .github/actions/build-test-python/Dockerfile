FROM ghcr.io/aica-technology/control-libraries/development-dependencies:latest

# FIXME (#70): remove the following line once the base image was updated
RUN pip3 install pytest

# Copy and set the entrypoint commands to execute when the container starts
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
