# Build environment for the SnapperGPS firmware (firmware_versions/*).
#
# Pins the exact Arm GNU Toolchain version referenced by the Makefiles
# (gcc-arm-none-eabi-10-2020-q4-major / 10.2.1). Newer major versions are not
# guaranteed to fit the firmware's 48 KB flash budget, see README.md.
#
# The toolchain has no native linux/arm64 build for this old release, so this
# image is pinned to linux/amd64 (runs fine under emulation on Apple Silicon).

FROM --platform=linux/amd64 ubuntu:20.04

ARG TOOLCHAIN_URL=https://developer.arm.com/-/media/Files/downloads/gnu-rm/10-2020q4/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2

RUN apt-get update \
    && apt-get install -y --no-install-recommends wget bzip2 make ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN wget -qO /tmp/toolchain.tar.bz2 "$TOOLCHAIN_URL" \
    && mkdir -p /opt/toolchain \
    && tar -xjf /tmp/toolchain.tar.bz2 -C /opt/toolchain \
    && rm /tmp/toolchain.tar.bz2

# firmware_versions/snapper/build/Makefile builds TOOLPATH from these two.
ENV TOOLCHAIN_PATH=/opt/toolchain
ENV TOOLCHAIN_VERSION=gcc-arm-none-eabi-10-2020-q4-major
# firmware_versions/snapper-accelerometer and snapper-pressure-sensor
# Makefiles use TOOLPATH directly instead.
ENV TOOLPATH=/opt/toolchain/gcc-arm-none-eabi-10-2020-q4-major/bin/

WORKDIR /repo

CMD ["bash"]
