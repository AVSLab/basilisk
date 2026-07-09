.. _bskSdkReleaseGuide:

BSK-SDK Release Guide
=====================

The `bsk-sdk <https://github.com/AVSLab/bsk_sdk>`_ package vendors the Basilisk
SDK headers and runtime for plugin authors. Its version is kept in sync with
Basilisk, so a new BSK release requires a corresponding SDK release.

The following steps are to be done after the Basilisk ``v2.X.Y`` release is fully completed.
This ensures that the Basilisk release is completed and available on the expected Basilisk branch.

.. _bsk-sdk-local-testing:

Testing the SDK Locally
-----------------------
Before pushing the ``bsk-sdk`` release branch to GitHub, test the SDK against
the matching Basilisk release locally.  This reproduces the important parts of
the GitHub Actions workflow: syncing the vendored SDK files, building the SDK
wheel, installing the matching Basilisk wheel, and building the example plugin.

Create a clean Python test environment inside your ``bsk_sdk`` folder using:

.. code-block:: bash

   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip
   python -m pip install build pytest scikit-build-core "cmake>=3.26" "ninja>=1.5"

In the ``bsk-sdk`` repository, check out the matching Basilisk release in the
local ``external/basilisk`` checkout and sync the SDK payload:

.. code-block:: bash

   git -C external/basilisk fetch --tags
   git -C external/basilisk checkout v2.X.Y
   python tools/sync_all.py

Verify that the SDK now records the expected Basilisk version:

.. code-block:: bash

   cat external/basilisk/docs/source/bskVersion.txt
   cat src/bsk_sdk/_bsk_version.txt
   grep '^version =' pyproject.toml

All three should refer to the same release version, e.g. ``2.X.Y``.  The
``pyproject.toml`` value is the published ``bsk-sdk`` version, while
``src/bsk_sdk/_bsk_version.txt`` is the Basilisk version the SDK was synced
from and the version CI will use to clone Basilisk.

Build and install the SDK wheel in the clean Python environment.  With no
output directory specified, ``python -m build`` writes wheels to the default
``dist/`` directory:

.. code-block:: bash

   python -m build --wheel
   python -m pip install --force-reinstall dist/bsk_sdk-*.whl
   python -m pytest tests/test_smoke.py -v

Install the matching Basilisk package set.  For a final release already
published to PyPI, use:

.. code-block:: bash

   python -m pip install --force-reinstall "bsk[all]==2.X.Y"

For a release candidate published to TestPyPI, use:

.. code-block:: bash

   python -m pip install --pre --force-reinstall \
     --index-url https://test.pypi.org/simple/ \
     --extra-index-url https://pypi.org/simple/ \
     "bsk[all]==2.X.YrcN"

Finally, build and test the example plugin against the locally installed SDK
wheel and matching Basilisk package.  The plugin wheel is written to the
example plugin project's default ``dist/`` directory:

.. code-block:: bash

   python -m build --wheel --no-isolation examples/custom-atm-plugin
   python -m pip install --force-reinstall examples/custom-atm-plugin/dist/*.whl
   python -m pytest examples/custom-atm-plugin/customExponentialAtmosphere/_UnitTest/test_customExponentialAtmosphere.py -v

.. note::

   The ``external/basilisk`` checkout is the local source input for
   ``tools/sync_all.py``.  Checking out a tag there does not affect published
   wheels by itself; CI and publishing use the committed ``bsk-sdk`` state.
   For a release-prep PR, commit the updated ``external/basilisk`` submodule
   pointer along with the synced files so the repository records which Basilisk
   source tree the SDK was synced from.  The release-relevant files are
   especially ``src/bsk_sdk/_bsk_version.txt``, the vendored files under
   ``src/bsk_sdk/``, and the SDK version in ``pyproject.toml``.


Major SDK Release
-----------------
To release a major version ``2.X.0`` the following steps are used:

#. Create a release branch from ``develop`` in the ``bsk-sdk`` repository.

#. Check out the matching Basilisk release tag in the local
   ``external/basilisk`` checkout and sync the SDK artifacts:

   .. code-block:: bash

      git -C external/basilisk fetch --tags
      git -C external/basilisk checkout v2.X.0
      python tools/sync_all.py

#. Update the ``version`` field in ``pyproject.toml`` to match the new
   major BSK release, e.g. ``2.X.0``.

#. Verify that the synced Basilisk version, SDK package version, and submodule
   pointer all refer to the same release:

   .. code-block:: bash

      cat external/basilisk/docs/source/bskVersion.txt
      cat src/bsk_sdk/_bsk_version.txt
      grep '^version =' pyproject.toml
      git -C external/basilisk describe --tags --exact-match

   These should all report ``2.X.0`` / ``v2.X.0``.

#. Run the :ref:`Testing the SDK Locally <bsk-sdk-local-testing>` steps before pushing the branch.

#. Commit the release-prep changes.  This normally includes the
   ``external/basilisk`` submodule pointer, ``pyproject.toml``,
   ``src/bsk_sdk/_bsk_version.txt``, and any tracked synced SDK artifacts
   updated by ``tools/sync_all.py``.

#. Push the branch to ``origin`` and create a PR to trigger the CI workflow.
   CI reads ``src/bsk_sdk/_bsk_version.txt`` to select the matching Basilisk
   release, so no ``ci.yml`` edit is needed for a normal major release.

#. Merge the PR to ``develop`` once CI tests pass.

#. Merge ``develop`` into ``master``.

#. Add the tag ``v2.X.0`` to ``master`` and push the tag to ``origin`` to
   trigger the wheel build and PyPI publish via GitHub Actions.

#. Create a Release on GitHub.

Patch Release
-------------
To release a patch version ``2.X.Y`` the following steps are used:

#. If this is the first SDK patch since ``v2.X.0``, create a
   ``patch/v2_X_x`` branch in the ``bsk-sdk`` repository from the
   ``v2.X.0`` SDK tag.  For later patches, use the existing
   ``patch/v2_X_x`` branch or branch from the latest ``v2.X.Y`` SDK tag.

#. Cherry-pick any SDK-specific fixes from ``develop`` that are required for
   this patch release.

#. Check out the matching Basilisk patch release tag in the local
   ``external/basilisk`` checkout and sync the SDK artifacts:

   .. code-block:: bash

      git -C external/basilisk fetch --tags
      git -C external/basilisk checkout v2.X.Y
      python tools/sync_all.py

#. Update the ``version`` field in ``pyproject.toml`` to match the patch BSK
   release, e.g. ``2.X.Y``.

#. Verify that the synced Basilisk version, SDK package version, and submodule
   pointer all refer to the same release:

   .. code-block:: bash

      cat external/basilisk/docs/source/bskVersion.txt
      cat src/bsk_sdk/_bsk_version.txt
      grep '^version =' pyproject.toml
      git -C external/basilisk describe --tags --exact-match

   These should all report ``2.X.Y`` / ``v2.X.Y``.

#. Run the :ref:`Testing the SDK Locally <bsk-sdk-local-testing>` steps before
   pushing the branch.

#. Commit the patch-release changes.  This normally includes the
   ``external/basilisk`` submodule pointer, ``pyproject.toml``,
   ``src/bsk_sdk/_bsk_version.txt``, and any tracked synced SDK artifacts
   updated by ``tools/sync_all.py``.

#. Push the branch to ``origin`` and manually run the ``CI`` action on
   https://github.com/AVSLab/bsk_sdk/actions. Wait for tests to complete.

#. Add the tag ``v2.X.Y`` to ``patch/v2_X_x`` and push the tag to ``origin`` to
   trigger the wheel build and PyPI publish via GitHub Actions.

#. Create a Release on GitHub.

No ``ci.yml`` edit is needed for a normal SDK patch release. CI reads
``src/bsk_sdk/_bsk_version.txt`` to select the matching Basilisk release.

.. note::

   The BSK SDK patch release should be prepared after the corresponding
   Basilisk patch release is tagged and its ``bsk[all]`` wheels are available.
   This lets local testing and CI install ``bsk[all]==2.X.Y`` without using
   temporary branch-specific workflow edits.


Testing SDK Changes Against a Basilisk Development Branch
---------------------------------------------------------
Use this workflow when developing or reviewing ``bsk-sdk`` changes against a
Basilisk branch that has not been released yet, such as ``develop`` or a
``feature/branch_name`` branch.  This is a local developer validation workflow
only.  Release SDK wheels should still be synced from tagged Basilisk releases.

The important rule is that the installed Basilisk Python package and the
Basilisk source tree used by ``tools/sync_all.py`` must come from the same
branch or commit.

Create a clean Python test environment in the ``bsk_sdk`` repository:

.. code-block:: bash

   python3 -m venv .venv
   source .venv/bin/activate
   python -m pip install --upgrade pip
   python -m pip install build pytest scikit-build-core "cmake>=3.26" "ninja>=1.5"

Check out the Basilisk branch you want to test.  This can be either the
``external/basilisk`` checkout or a separate Basilisk clone.  A separate clone is
often cleaner for exploratory testing because it does not move the SDK
submodule pointer.

.. code-block:: bash

   git -C ../basilisk fetch origin
   git -C ../basilisk checkout feature/branch_name
   git -C ../basilisk pull --ff-only

Build and install a local Basilisk wheel from that branch into the SDK test
environment:

.. code-block:: bash

   python -m pip wheel --no-deps -v -w /tmp/bsk-dev-wheel ../basilisk
   python -m pip install --force-reinstall /tmp/bsk-dev-wheel/bsk-*.whl

If the plugin being tested needs optional Basilisk components such as OpNav,
build Basilisk with the matching ``CONAN_ARGS`` or install matching optional
component wheels from the same Basilisk branch.

Sync ``bsk-sdk`` from the same Basilisk source tree:

.. code-block:: bash

   python tools/sync_all.py --basilisk-root ../basilisk

Verify that the installed Basilisk package and the synced SDK version agree:

.. code-block:: bash

   python -c "import Basilisk, bsk_sdk; print('Basilisk:', Basilisk.__version__); print('SDK synced from:', bsk_sdk.bsk_version())"
   cat ../basilisk/docs/source/bskVersion.txt
   cat src/bsk_sdk/_bsk_version.txt

Build and test the SDK wheel:

.. code-block:: bash

   python -m build --wheel
   python -m pip install --force-reinstall dist/bsk_sdk-*.whl
   python -m pytest tests/test_smoke.py -v

Finally, build and test the example plugin against the locally installed
Basilisk and SDK wheels:

.. code-block:: bash

   python -m build --wheel --no-isolation examples/custom-atm-plugin
   python -m pip install --force-reinstall examples/custom-atm-plugin/dist/*.whl
   python -m pytest examples/custom-atm-plugin/customExponentialAtmosphere/_UnitTest/test_customExponentialAtmosphere.py -v

For a real plugin under development, replace ``examples/custom-atm-plugin`` with
the plugin repository path and run that plugin's own test suite.

.. note::

   If this was only an exploratory compatibility test, do not commit the synced
   SDK artifacts or submodule pointer.  Commit those changes only when preparing
   an intentional SDK branch that should track the tested Basilisk branch or
   release.
