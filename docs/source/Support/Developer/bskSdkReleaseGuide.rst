.. _bskSdkReleaseGuide:

BSK-SDK Release Guide
=====================

The `bsk-sdk <https://github.com/AVSLab/bsk_sdk>`_ package vendors the Basilisk
SDK headers and runtime for extension authors. Its version is kept in sync with
Basilisk, so a new Basilisk release requires a corresponding SDK release.

This guide separates the release workflows from the common build and test
tasks. Start with the appropriate workflow, then follow its links to the
detailed task procedures.

Release Model
-------------

An SDK build involves three related versions:

* the Basilisk source used by ``tools/sync_all.py``;
* the versions recorded in ``src/bsk_sdk/_bsk_version.txt`` and
  ``pyproject.toml``; and
* the Basilisk Python package installed while testing the SDK and example
  extension.

These versions must describe the same Basilisk release, branch, or commit. The
source checkout is only an input to ``tools/sync_all.py``. CI and publishing
use the synced files committed to the ``bsk-sdk`` repository, not the
developer's local Basilisk checkout.

Use the following table to select a workflow.

.. list-table:: SDK workflows
   :header-rows: 1
   :widths: 20 23 27 30

   * - Workflow
     - Basilisk source
     - Installed Basilisk
     - SDK branch and tag
   * - Major release
     - ``v2.X.0`` tag
     - PyPI ``2.X.0``
     - Release branch, then ``v2.X.0``
   * - Patch release
     - ``v2.X.Y`` tag
     - PyPI ``2.X.Y``
     - ``patch/v2_X_x``, then ``v2.X.Y``
   * - Beta-cycle validation
     - ``develop`` or matching beta branch
     - Matching local or nightly build
     - Beta SDK branch
   * - Feature-branch validation
     - Exact feature branch or commit
     - Locally built from the same source
     - Temporary SDK test branch

Final and patch SDK releases must be prepared after the corresponding Basilisk
release is tagged and its ``bsk[all]`` wheels are available. During a beta
cycle, the SDK can instead be synced from Basilisk ``develop`` or a matching
feature branch using a PEP 440 pre-release version such as ``2.X.0bN``.

Release Workflows
-----------------

.. _bsk-sdk-major-release:

Major SDK Release
~~~~~~~~~~~~~~~~~

Use this workflow to release ``2.X.0`` after the corresponding Basilisk
release is complete.

#. Create an SDK release branch from ``develop``.
#. :ref:`Create a clean test environment <bsk-sdk-task-environment>`.
#. :ref:`Select the Basilisk source <bsk-sdk-task-select-source>` and check out
   the ``v2.X.0`` tag.
#. :ref:`Sync the SDK payload <bsk-sdk-task-sync>` from that checkout.
#. :ref:`Verify versions and provenance <bsk-sdk-task-verify>`. All version
   values must report ``2.X.0``, and the source checkout must report
   ``v2.X.0``.
#. :ref:`Install Basilisk from PyPI <bsk-sdk-install-final>` using
   ``bsk[all]==2.X.0``.
#. :ref:`Build and test the SDK wheel <bsk-sdk-task-build-sdk>`.
#. :ref:`Build and test the example extension <bsk-sdk-task-test-extension>`.
#. :ref:`Commit the synced payload <bsk-sdk-task-commit>` and open a PR to
   ``develop``. Wait for CI to pass before merging.
#. Merge ``develop`` into ``master``.
#. Tag ``master`` with ``v2.X.0`` and push the tag. The tag triggers the wheel
   build and PyPI publication through GitHub Actions.
#. Create the corresponding GitHub Release.

.. _bsk-sdk-patch-release:

Patch SDK Release
~~~~~~~~~~~~~~~~~

Use this workflow to release ``2.X.Y`` after the corresponding Basilisk patch
release is complete.

#. For the first SDK patch after ``v2.X.0``, create ``patch/v2_X_x`` from the
   ``v2.X.0`` SDK tag. For later patches, use the existing patch branch or
   branch from the latest ``v2.X.Y`` SDK tag.
#. Cherry-pick any required SDK-specific fixes from ``develop``.
#. :ref:`Create a clean test environment <bsk-sdk-task-environment>`.
#. :ref:`Select the Basilisk source <bsk-sdk-task-select-source>` and check out
   the ``v2.X.Y`` tag.
#. :ref:`Sync the SDK payload <bsk-sdk-task-sync>` from that checkout.
#. :ref:`Verify versions and provenance <bsk-sdk-task-verify>`. All version
   values must report ``2.X.Y``, and the source checkout must report
   ``v2.X.Y``.
#. :ref:`Install Basilisk from PyPI <bsk-sdk-install-final>` using
   ``bsk[all]==2.X.Y``.
#. :ref:`Build and test the SDK wheel <bsk-sdk-task-build-sdk>`.
#. :ref:`Build and test the example extension <bsk-sdk-task-test-extension>`.
#. :ref:`Commit the synced payload <bsk-sdk-task-commit>` and push the patch
   branch.
#. Manually run the ``CI`` action from the
   `bsk-sdk Actions page <https://github.com/AVSLab/bsk_sdk/actions>`_ and wait
   for it to pass.
#. Tag ``patch/v2_X_x`` with ``v2.X.Y`` and push the tag. The tag triggers the
   wheel build and PyPI publication through GitHub Actions.
#. Create the corresponding GitHub Release.

.. _bsk-sdk-beta-validation:

Beta and Branch Validation
~~~~~~~~~~~~~~~~~~~~~~~~~~

Beta-cycle and feature-branch testing share the same local tasks but have
different goals.

**Beta-cycle validation** prepares an SDK branch from Basilisk ``develop`` or
a matching beta branch. The Basilisk version file must contain a PEP 440
pre-release version such as ``2.X.0bN``. CI recognizes ``aN`` and ``bN`` as
development versions, checks out Basilisk ``develop``, and installs the
nightly ``bsk[all]`` package set.

**Feature-branch validation** checks compatibility with an unreleased Basilisk
change. The installed Basilisk wheel and the source passed to
``tools/sync_all.py`` must come from the same branch or commit. This is a local
validation workflow; it does not prepare a release wheel.

For either mode:

#. Create or check out the appropriate SDK test branch.
#. :ref:`Create a clean test environment <bsk-sdk-task-environment>`.
#. :ref:`Select the Basilisk source <bsk-sdk-task-select-source>` and check out
   ``develop``, the beta branch, or the feature branch being tested.
#. For feature-branch testing, record the exact Basilisk commit so the result
   can be reproduced.
#. Install the matching Basilisk package. For beta-cycle validation against
   current ``develop``, use the :ref:`nightly develop wheel
   <bsk-sdk-install-nightly>`. For feature branches, or whenever the nightly
   wheel does not match the selected source, :ref:`build and install Basilisk
   locally <bsk-sdk-install-local>`.
#. :ref:`Sync the SDK payload <bsk-sdk-task-sync>` from the same checkout.
#. :ref:`Verify versions and provenance <bsk-sdk-task-verify>`.
#. :ref:`Build and test the SDK wheel <bsk-sdk-task-build-sdk>`.
#. :ref:`Build and test the example extension
   <bsk-sdk-task-test-extension>`, or substitute the extension under
   development and run its test suite.
#. For a beta SDK branch, :ref:`commit the synced payload
   <bsk-sdk-task-commit>` and open a PR. For exploratory feature-branch
   testing, do not commit the synced payload or moved submodule pointer.

When Basilisk reaches a release candidate or final release, sync the SDK again
from the corresponding Basilisk tag and repeat the applicable release
workflow.

.. _bsk-sdk-local-testing:

Common Tasks
------------

The following procedures are shared by the release and validation workflows.
Run all commands from the root of the ``bsk_sdk`` repository unless noted
otherwise.

.. _bsk-sdk-task-environment:

Create a Clean Test Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create the environment inside the ``bsk_sdk`` repository:

.. code-block:: bash

   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip
   python -m pip install build pytest scikit-build-core "cmake>=3.26" "ninja>=1.5"

Use a newly created environment for each release validation so previously
installed Basilisk or SDK packages cannot mask missing dependencies.

.. _bsk-sdk-task-select-source:

Select the Basilisk Source
~~~~~~~~~~~~~~~~~~~~~~~~~~

Set ``BSK_ROOT`` to either the SDK repository's Basilisk submodule or an
existing local Basilisk checkout.

To use the submodule, initialize it once after cloning ``bsk-sdk``:

.. code-block:: bash

   git submodule update --init --recursive external/basilisk
   BSK_ROOT=external/basilisk

To use an existing checkout without moving the SDK submodule pointer:

.. code-block:: bash

   BSK_ROOT=~/Repos/basilisk

For a tagged release, fetch tags and check out the required tag:

.. code-block:: bash

   git -C "$BSK_ROOT" fetch --tags
   git -C "$BSK_ROOT" checkout v2.X.Y

For beta-cycle or feature-branch testing, fetch and check out the required
branch:

.. code-block:: bash

   git -C "$BSK_ROOT" fetch origin
   git -C "$BSK_ROOT" checkout feature/branch_name
   git -C "$BSK_ROOT" pull --ff-only
   git -C "$BSK_ROOT" rev-parse HEAD

Use ``develop`` in place of ``feature/branch_name`` for a normal beta cycle.

.. _bsk-sdk-task-sync:

Sync the SDK Payload
~~~~~~~~~~~~~~~~~~~~

Sync the vendored headers, runtime support, and version metadata from the
selected Basilisk source:

.. code-block:: bash

   python tools/sync_all.py --basilisk-root "$BSK_ROOT"

The script updates the vendored files under ``src/bsk_sdk/``, including
``src/bsk_sdk/_bsk_version.txt``, and the ``[project].version`` field in
``pyproject.toml``.

.. _bsk-sdk-task-verify:

Verify Versions and Provenance
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Compare the Basilisk source version with both SDK version records:

.. code-block:: bash

   cat "$BSK_ROOT/docs/source/bskVersion.txt"
   cat src/bsk_sdk/_bsk_version.txt
   python - <<'PY'
   from pathlib import Path
   import re
   text = Path("pyproject.toml").read_text()
   print(re.search(r'(?ms)^\[project\].*?^version = "([^"]+)"', text).group(1))
   PY

All three values must match. For a release, also confirm that the checkout is
at the expected tag:

.. code-block:: bash

   git -C "$BSK_ROOT" describe --tags --exact-match

.. _bsk-sdk-task-install-basilisk:

Install the Matching Basilisk Package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Choose one installation method based on the workflow. The installed package
must correspond to the source used to sync the SDK.

.. _bsk-sdk-install-final:

Final Release from PyPI
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   python -m pip install --force-reinstall "bsk[all]==2.X.Y"

.. _bsk-sdk-install-candidate:

Release Candidate from TestPyPI
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   python -m pip install --pre --force-reinstall \
     --index-url https://test.pypi.org/simple/ \
     --extra-index-url https://pypi.org/simple/ \
     "bsk[all]==2.X.YrcN"

.. _bsk-sdk-install-nightly:

Develop Nightly Wheel
^^^^^^^^^^^^^^^^^^^^^

For beta-cycle validation against the current Basilisk ``develop`` branch,
install the latest nightly development wheel:

.. code-block:: bash

   python -m pip install --pre --upgrade --force-reinstall --no-cache-dir \
     --index-url https://avslab.github.io/basilisk/nightly/ \
     --extra-index-url https://pypi.org/simple/ \
     "bsk[all]"

The nightly index supplies the Basilisk development wheels, while the PyPI
index supplies third-party dependencies. Confirm that the installed version
matches the version in the Basilisk source selected for ``tools/sync_all.py``.
If it does not match, use a local build from that exact source instead.

.. _bsk-sdk-install-local:

Local Beta or Feature Branch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build Basilisk from ``BSK_ROOT`` and install that wheel into the SDK test
environment:

.. code-block:: bash

   python -m pip wheel --no-deps -v -w /tmp/bsk-dev-wheel "$BSK_ROOT"
   python -m pip install --force-reinstall /tmp/bsk-dev-wheel/bsk-*.whl

If the extension needs optional Basilisk components such as OpNav, build
Basilisk with matching ``CONAN_ARGS`` or install matching optional-component
wheels produced from the same Basilisk source.

.. _bsk-sdk-task-build-sdk:

Build and Test the SDK Wheel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Build and install the SDK wheel, then run the SDK test suite:

.. code-block:: bash

   python -m build --wheel
   python -m pip install --force-reinstall dist/bsk_sdk-*.whl
   python -m pytest tests -v
   python -c "import Basilisk, bsk_sdk; print('Basilisk:', Basilisk.__version__); print('SDK synced from:', bsk_sdk.bsk_version())"

.. _bsk-sdk-task-test-extension:

Build and Test the Example Extension
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Build the example against the installed SDK and Basilisk wheels, install it,
verify its runtime imports, and run all example tests:

.. code-block:: bash

   python -m build --wheel --no-isolation examples/custom-atm-extension
   python -m pip install examples/custom-atm-extension/dist/*.whl
   python -c "import Basilisk, numba, custom_atm; from custom_atm import numbaAtmosphere"
   python -m pytest examples -v

For an extension under development, replace ``examples/custom-atm-extension``
with that extension's repository path and run its own test suite.

.. _bsk-sdk-task-commit:

Commit the Synced Payload
~~~~~~~~~~~~~~~~~~~~~~~~~

Review and commit the release-preparation changes produced by
``tools/sync_all.py``. These normally include:

* ``pyproject.toml``;
* ``src/bsk_sdk/_bsk_version.txt``; and
* updated vendored SDK artifacts under ``src/bsk_sdk/``.

If the release branch intentionally records Basilisk provenance through
``external/basilisk``, commit the updated submodule pointer as well. A checkout
selected with ``--basilisk-root`` does not otherwise alter the published SDK.

CI and Publishing Behavior
--------------------------

CI reads ``src/bsk_sdk/_bsk_version.txt`` to select the matching Basilisk
build. Versions containing ``aN`` or ``bN`` are treated as development builds;
CI checks out Basilisk ``develop`` and installs the nightly ``bsk[all]``
package set. Final and patch versions select the corresponding published
Basilisk release.

No temporary ``ci.yml`` edit is needed for normal beta, major, or patch SDK
workflows. A pushed SDK version tag triggers the GitHub Actions wheel build and
PyPI publication. Always wait for the corresponding Basilisk release and
package publication before preparing a final or patch SDK tag.
