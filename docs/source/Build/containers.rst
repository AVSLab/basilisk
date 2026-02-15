.. _bskContainers:

Containers
==========

.. warning::

    :beta:`Container Support` This feature is currently in beta.

Basilisk publishes prebuilt container images from CI to:

- ``ghcr.io/avslab/basilisk``

Images are published as multi-architecture manifests for:

- ``linux/amd64``
- ``linux/arm64``


Tag and Versioning Policy
-------------------------

Container tags follow the CI release tag publishing policy:

- Push a release tag matching ``v*`` publishes:

  - ``vX.Y.Z`` (or whatever the pushed tag name is)
  - ``latest``

Use specific version tags for reproducible production usage.


Pull Examples
-------------

Pull from GHCR::

    docker pull ghcr.io/avslab/basilisk:latest
    docker pull ghcr.io/avslab/basilisk:v2.9.0


Run Example
-----------

Run a quick import check::

    docker run --rm ghcr.io/avslab/basilisk:latest

Open an interactive shell::

    docker run --rm -it ghcr.io/avslab/basilisk:latest bash

Run with Vizard connectivity (exposes ports 5556 and 5570)::

    docker run --rm -it -p 5556:5556 -p 5570:5570 ghcr.io/avslab/basilisk:latest bash

Basilisk binds to all network interfaces (``0.0.0.0``) by default, allowing Vizard running
on your host machine to connect via ``tcp://localhost:5556``. The Docker port forwarding
maps the container's ports 5556 and 5570 to your host machine.

Run with Bokeh visualization support (for Monte Carlo scenarios, adds port 5006)::

    docker run --rm -it -p 5556:5556 -p 5570:5570 -p 5006:5006 ghcr.io/avslab/basilisk:latest bash


Building Locally
----------------

To build from your current local checkout, use the repository ``Dockerfile``::

    docker build -t bsk:local .

This produces a slim runtime image containing only the installed Basilisk wheel
and the shared libraries it needs at runtime.

By default the build enables the following options:

- ``--opNav True``
- ``--mujoco True``
- ``--mujocoReplay True``

You can override these at build time::

    docker build --build-arg CONAN_ARGS="--opNav True --mujoco False --mujocoReplay False" -t bsk:local .

To build a development image with the full toolchain (compiler, cmake, swig,
source tree, etc.), target the ``builder`` stage::

    docker build --target builder -t bsk-dev:local .
