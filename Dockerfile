# Stage 1: Basic Setup
FROM python:3.12-slim as base
WORKDIR /usr/src/app

RUN apt-get update && apt-get install -y \
    cmake \
    git \
    build-essential \
    python3-setuptools \
    python3-dev \
    python3-tk \
    python3-pip \
    swig \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip && \
    pip install wheel \
    'conan<2.0' \
    pandas numpy matplotlib pytest Pillow 'parse>=1.18.0' \
    cmake

# Copy the Basilisk project files into the container
COPY . .

# Stage 2: Conan Build (Checkpoint)
FROM base as conan-build
RUN python3 conanfile.py

RUN pip install -e .
# Optionally, you can create a checkpoint image here by tagging this stage in your build command.

# Stage 3: Final Image
FROM conan-build as final

# Set the default command, if necessary.
CMD ["python3", "examples/scenarioBasicOrbit.py"]
