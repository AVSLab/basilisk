# syntax=docker/dockerfile:1.7
#
# Multi-stage Dockerfile for Basilisk:
# - "builder" stage: build deps + conan + swig + compilation
# - "runtime" stage: minimal runtime libs + the installed Basilisk wheel
#
# Build (runtime image):
#   docker build -t bsk:py312 .
#
# Build (dev image with full toolchain):
#   docker build --target builder -t bsk-dev:py312 .
#
# Run:
#   docker run --rm bsk:py312
#
# Optional build args:
#   --build-arg PYTHON_VERSION=3.12
#   --build-arg CONAN_ARGS="--opNav True --mujoco True --mujocoReplay True"

ARG PYTHON_VERSION=3.13

# Builder
FROM python:${PYTHON_VERSION}-slim AS builder

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /opt/basilisk

# Build toolchain + headers needed to compile Basilisk.
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    swig \
    python3-setuptools \
    python3-tk \
    libgl1-mesa-dev \
    libglfw3-dev \
    libgtk2.0-dev \
    libx11-dev \
    libxcursor-dev \
    libxi-dev \
    libxinerama-dev \
    libxrandr-dev \
    && rm -rf /var/lib/apt/lists/*

RUN python -m pip install --upgrade pip uv

# Configure Conan to allow system package manager usage.
RUN mkdir -p /root/.conan2 \
    && printf "tools.system.package_manager:mode=install\n" >> /root/.conan2/global.conf \
    && printf "tools.system.package_manager:sudo=False\n" >> /root/.conan2/global.conf

# Copy source
COPY . .

# Build Basilisk wheel.
ARG CONAN_ARGS="--opNav True --mujoco True --mujocoReplay True"

RUN --mount=type=cache,target=/root/.cache/pip \
    --mount=type=cache,target=/root/.cache/uv \
    CONAN_ARGS="${CONAN_ARGS}" \
    uv pip install --system build && \
    python -m build --wheel --outdir /opt/wheels .

# Runtime
FROM python:${PYTHON_VERSION}-slim AS runtime
ARG PYTHON_VERSION=3.12

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PIP_NO_CACHE_DIR=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1

WORKDIR /opt/app

# Runtime libraries only
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-tk \
    libgtk2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install Basilisk wheel, strip binaries, and clean up.
COPY --from=builder /opt/wheels /opt/wheels
RUN apt-get update && apt-get install -y --no-install-recommends binutils \
    && python -m pip install --no-cache-dir /opt/wheels/*.whl \
    && rm -rf /opt/wheels \
    # Remove unnecessary files to reduce image size
    && find /usr/local/lib/python* -type d -name 'tests' -exec rm -rf {} + 2>/dev/null || true \
    && find /usr/local/lib/python* -type d -name '__pycache__' -exec rm -rf {} + 2>/dev/null || true \
    && find /usr/local/lib/python* -name '*.pyc' -delete \
    && find /usr/local/lib/python* -name '*.pyo' -delete \
    # Remove static libraries
    && find /usr/local/lib/python* -name '*.a' -delete \
    # Strip debug symbols from shared objects
    && find /usr/local/lib/python* -name '*.so' -exec strip --strip-unneeded {} + \
    # Remove pip
    && python -m pip uninstall -y pip \
    && apt-get purge -y binutils && apt-get autoremove -y \
    && rm -rf /var/lib/apt/lists/*

# Drop privileges
RUN useradd --create-home --shell /bin/bash basilisk
USER basilisk
WORKDIR /home/basilisk

# Expose Vizard communication ports and Bokeh server port
EXPOSE 5556 5570 5006

CMD ["python", "-c", "import Basilisk; print('Basilisk import OK')"]
