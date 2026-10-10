.. _bskSdkReleaseGuide:

BSK-SDK Release Guide
=====================

The `bsk-sdk <https://github.com/AVSLab/bsk_sdk>`_ package supplies Basilisk
headers, runtime sources, and build helpers for external extensions. Its
package version follows the Basilisk source from which it is synchronized.

To update BSK-SDK, select the intended Basilisk checkout and run
``tools/sync_all.py --refresh-example`` with the source options described below.
This updates the SDK version, example requirements, Rust manifests, lockfile,
and license report together. Review the generated changes; no manual version
edits are needed in ``examples/custom-atm-extension``.

What Changes When the Version Changes
-------------------------------------

The version originates in Basilisk's ``docs/source/bskVersion.txt``. Basilisk
maintainers update that file as part of a Basilisk beta or release change.
For an SDK update, select the corresponding committed source; do not change
the Basilisk version file just to obtain a different SDK version.

The following table lists the SDK files to review, not files to edit by hand.
``EXAMPLE`` abbreviates
``examples/custom-atm-extension``; all other paths are relative to the SDK
repository root.

.. list-table:: Version-update files
   :header-rows: 1
   :widths: 42 58

   * - File or Git entry
     - How it is updated
   * - ``external/basilisk``
     - Select and stage the intended Basilisk commit. This is a Git submodule
       pointer, not a version text file.
   * - ``src/bsk_sdk/_bsk_version.txt``
     - ``tools/sync_all.py`` copies the selected Basilisk version.
   * - ``pyproject.toml``
     - The sync command sets ``[project].version`` to that same version.
   * - ``EXAMPLE/pyproject.toml``
     - The sync command updates the build requirements for ``bsk-sdk`` and
       ``bsk``, plus the runtime requirement for ``bsk``.
   * - ``EXAMPLE/Cargo.toml``
     - ``sync_rust.py``, called by ``sync_all.py``, updates the Rust minimum,
       support-crate versions, and ``bsk-build``/``bsk-messages`` sources.
   * - ``EXAMPLE/rustAtmosphere/Cargo.toml``
     - The same sync updates the build dependency on ``bsk-build``.
   * - ``EXAMPLE/Cargo.lock``
     - ``--refresh-example`` regenerates it with Cargo after synchronization
       to resolve the selected dependency sources and versions.
   * - ``EXAMPLE/custom_atm/RUST-THIRD-PARTY.txt``
     - ``--refresh-example`` regenerates it from the locked dependencies
       using the synchronized license generator and pinned tool version.

Use the sync command to update these fields. The example package's own
``[project].version`` and the Rust module's ``[package].version`` are separate
from the BSK version. Rust support-crate versions come from Basilisk's Rust
support metadata; they are not changed to ``2.X.Y``. Normal version updates
do not require edits to CMake helpers, ``src/bsk_sdk/__init__.py``, or CI YAML.

Sync also recreates the ignored headers, runtime, SWIG, Rust support, and
message-generation files under ``src/bsk_sdk/`` and ``tools/msgAutoSource/``.
These files are build inputs included in distributions, but are not normally
committed to Git. Do not force-add the ignored directories.

Choose a Workflow
-----------------

.. list-table:: Source and publication policy
   :header-rows: 1
   :widths: 22 43 35

   * - Workflow
     - Basilisk source and Python package
     - SDK destination
   * - Next beta cycle
     - Latest develop nightly wheel and its exact source commit
     - Beta branch, then ``develop``; no package publication
   * - Major release, ``2.X.0``
     - Basilisk ``v2.X.0`` tag and matching PyPI wheels
     - Release branch through ``develop`` to ``master``; PyPI
   * - Patch release, ``2.X.Y``
     - Basilisk ``v2.X.Y`` tag and matching PyPI wheels
     - ``patch/v2_X_x``; PyPI
   * - Release candidate, ``2.X.YrcN``
     - Basilisk ``v2.X.YrcN`` tag and matching TestPyPI wheels
     - SDK release or patch branch; TestPyPI
   * - Feature-branch validation
     - Local Basilisk wheel built from the selected feature commit
     - Temporary SDK test branch; no package publication

Final and candidate SDK releases follow the corresponding Basilisk tag and
wheel publication. An SDK tag uses a leading ``v``; package versions do not.
The SDK publication workflow rejects alpha and beta versions, including when
triggered with a ``test*`` tag.

The SDK headers and the installed Basilisk native wheel must come from the
same source revision. Matching a version string such as ``2.13.0b0`` is not
sufficient: many develop commits can retain that version.

Release and Beta Workflows
--------------------------

.. _bsk-sdk-beta-validation:

Start the Next Beta Cycle
~~~~~~~~~~~~~~~~~~~~~~~~~

For example, after publishing BSK-SDK ``2.12.0``, prepare ``2.13.0b0`` once
Basilisk has committed that beta version and published its develop wheels.

#. Create a beta preparation branch from SDK ``develop``. Ensure any SDK
   release fixes made on ``master`` have also reached ``develop``.
#. :ref:`Create a clean environment <bsk-sdk-task-environment>` and
   :ref:`initialize the Basilisk submodule <bsk-sdk-task-select-source>`.
#. :ref:`Install the latest nightly and select its source commit
   <bsk-sdk-install-nightly>`. Confirm that Basilisk's version file contains
   the intended beta version. If the next beta is not available yet, complete
   that Basilisk update first rather than relabeling older sources.
#. Record this baseline for local SDK builds before syncing:

   .. code-block:: bash

      git add external/basilisk

#. :ref:`Sync and refresh in development mode <bsk-sdk-task-sync>`, then
   :ref:`review the refreshed example <bsk-sdk-task-rust-dependencies>`.
   Both the lockfile and license report can change when moving from release
   tags back to local Rust dependencies.
#. :ref:`Build the SDK <bsk-sdk-task-build-sdk>`,
   :ref:`verify versions and source revisions <bsk-sdk-task-verify>`, and
   :ref:`build and test the example extension <bsk-sdk-task-test-extension>`.
#. :ref:`Review and commit the preparation files <bsk-sdk-task-commit>`.
   Open a PR to ``develop`` and wait for CI before merging. Do not create a
   beta publication tag.

The recorded submodule commit provides a reproducible local starting point.
Beta CI and the scheduled nightly workflow select the latest available BSK
nightly and its matching sources each time they run. Updating that baseline
for every nightly is unnecessary.

.. _bsk-sdk-major-release:

Major SDK Release
~~~~~~~~~~~~~~~~~

Use this workflow for ``2.X.0`` after the corresponding Basilisk release has
been published.

#. Create an SDK release branch from ``develop``.
#. :ref:`Create a clean environment <bsk-sdk-task-environment>`, then
   :ref:`select and stage the Basilisk release tag
   <bsk-sdk-task-select-source>` with ``BSK_VERSION=2.X.0``.
#. :ref:`Install the matching PyPI package <bsk-sdk-install-final>`.
#. :ref:`Sync and refresh in release mode <bsk-sdk-task-sync>`, then
   :ref:`review the refreshed example <bsk-sdk-task-rust-dependencies>`
   against the public Basilisk tag.
#. :ref:`Build the SDK <bsk-sdk-task-build-sdk>`,
   :ref:`verify its version and source <bsk-sdk-task-verify>`, and
   :ref:`build and test the example extension <bsk-sdk-task-test-extension>`.
#. :ref:`Review and commit the preparation files <bsk-sdk-task-commit>`.
   Open a PR to ``develop``, wait for CI, and merge it.
#. Merge ``develop`` into ``master``. Tag the validated SDK commit on
   ``master`` as ``v2.X.0`` and push the tag.
#. Wait for ``Publish Wheels`` to succeed, verify the SDK distributions on
   PyPI, and create the GitHub Release. Then start the next beta cycle on
   ``develop``.

.. _bsk-sdk-patch-release:

Patch SDK Release
~~~~~~~~~~~~~~~~~

Use the same file-update and validation tasks as a major release, but preserve
the existing release line.

#. For the first patch, create ``patch/v2_X_x`` from SDK tag ``v2.X.0``.
   For later patches, continue that branch from the latest SDK patch release.
#. Bring in the required SDK fixes. Ensure fixes developed on the patch
   branch also reach ``develop``.
#. :ref:`Create a clean environment <bsk-sdk-task-environment>`,
   :ref:`select and stage Basilisk tag v2.X.Y <bsk-sdk-task-select-source>`,
   and :ref:`install bsk[all]==2.X.Y from PyPI <bsk-sdk-install-final>`.
#. :ref:`Sync and refresh in release mode <bsk-sdk-task-sync>`,
   :ref:`review the refreshed example <bsk-sdk-task-rust-dependencies>`,
   and complete the
   :ref:`SDK build <bsk-sdk-task-build-sdk>`,
   :ref:`version checks <bsk-sdk-task-verify>`, and
   :ref:`example tests <bsk-sdk-task-test-extension>`.
#. :ref:`Commit the preparation files <bsk-sdk-task-commit>` and push the
   patch branch. Manually run ``CI`` on that branch from the
   `SDK Actions page <https://github.com/AVSLab/bsk_sdk/actions>`_ and wait for
   it to pass; ordinary push CI targets ``master`` and ``develop``.
#. Tag the validated patch-branch commit as ``v2.X.Y`` and push the tag.
   Verify ``Publish Wheels`` and the PyPI artifacts, then create the GitHub
   Release.

.. _bsk-sdk-candidate-release:

Release Candidate
~~~~~~~~~~~~~~~~~

On the appropriate SDK release or patch branch, follow the same preparation
and validation tasks with ``BSK_VERSION=2.X.YrcN``. Select Basilisk tag
``v2.X.YrcN`` and :ref:`install from TestPyPI <bsk-sdk-install-candidate>`.
Use release-mode synchronization with ``--refresh-example`` so Cargo resolves
that public tag.

After validation and CI, tag the SDK commit ``v2.X.YrcN``. ``Publish Wheels``
routes it to TestPyPI. Preparing the subsequent final SDK release requires
another sync with ``--refresh-example`` from the final Basilisk tag;
removing ``rcN`` from one or two SDK files is not sufficient.

Feature-Branch Validation
~~~~~~~~~~~~~~~~~~~~~~~~~

For an unreleased Basilisk change, use a temporary SDK branch and a separate
Basilisk checkout. :ref:`Build Basilisk locally <bsk-sdk-install-local>` from
that checkout, sync the SDK from the same source with
``--local-rust-dependencies --refresh-example``, and run the
SDK and example checks. This also works when the feature checkout still has
an RC or final version string.

Do not commit machine-specific Cargo paths or an exploratory submodule move
as part of an SDK release. To prepare a reproducible beta or release branch,
use the recorded ``external/basilisk`` checkout described below.

.. _bsk-sdk-local-testing:

Common Tasks
------------

Run commands from the ``bsk_sdk`` repository root unless stated otherwise.
The examples use Bash syntax. Replace version placeholders such as
``2.X.Y`` with the actual version and stop if a command fails.

.. _bsk-sdk-task-environment:

Create a Clean Test Environment
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Choose a new environment directory for each validation. The temporary output
directory below keeps wheels and build caches from earlier runs out of the
installation commands.

.. code-block:: bash

   python3 -m venv env/sdk-validation
   source env/sdk-validation/bin/activate
   python -m pip install --upgrade pip
   python -m pip install build pytest scikit-build-core numba \
     "cmake>=3.26" "ninja>=1.5"
   CHECK_DIR="$(mktemp -d "${TMPDIR:-/tmp}/bsk-sdk-check.XXXXXX")"

On Windows, activate the environment using its ``Scripts`` directory. The
bundled extension also requires Rust; its required minimum is recorded in
``src/bsk_sdk/rust/support-versions.json`` after synchronization. CI checks
the minimum Rust toolchain as well as the supported Python/OS matrix.

.. _bsk-sdk-task-select-source:

Select and Record the Basilisk Source
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Start with a clean SDK branch and Basilisk checkout. To prepare a committed
beta or release update, initialize the submodule and select it as the source:

.. code-block:: bash

   git submodule update --init --recursive external/basilisk
   BSK_ROOT="$PWD/external/basilisk"

For a final or candidate release, select the already-published Basilisk tag
and stage the new submodule pointer:

.. code-block:: bash

   BSK_VERSION=2.X.Y
   git -C "$BSK_ROOT" fetch origin tag "v${BSK_VERSION}"
   git -C "$BSK_ROOT" checkout --detach "v${BSK_VERSION}"
   git add external/basilisk

Use ``2.X.0`` for a major release or ``2.X.YrcN`` for a candidate. For a beta
cycle, select the exact source revision reported by the installed nightly
using the :ref:`nightly installation task <bsk-sdk-install-nightly>`, then
stage the pointer as shown in the beta workflow.

Stage an intentional submodule update **before** running the normal sync.
Otherwise, sync restores the commit recorded in the SDK Git index and can
undo an unstaged checkout. Staging the pointer does not commit it or copy the
Basilisk repository into BSK-SDK.

For temporary feature work, select an existing checkout instead:

.. code-block:: bash

   BSK_ROOT=~/Repos/basilisk
   git -C "$BSK_ROOT" rev-parse HEAD

The second command prints the exact committed revision of that checkout.
``--basilisk-root`` preserves a checkout outside the SDK submodule. To test
an unrecorded commit inside ``external/basilisk``, add
``--no-sync-submodules`` to the sync command instead of staging the pointer.

.. _bsk-sdk-task-sync:

Synchronize the SDK and Refresh the Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For a final or candidate release:

.. code-block:: bash

   python tools/sync_all.py --basilisk-root "$BSK_ROOT" --refresh-example

For a beta cycle, nightly, or feature checkout:

.. code-block:: bash

   python tools/sync_all.py --basilisk-root "$BSK_ROOT" \
     --local-rust-dependencies --refresh-example

Both commands update the synchronized files in the version-update table.
The default Rust policy uses tags for RC/final versions and local paths for
alpha/beta versions. ``--local-rust-dependencies`` keeps Rust on the selected
checkout even when a nightly or feature build reports an RC/final version.
Use the default policy when preparing a package for publication.

``--refresh-example`` requires Cargo on ``PATH`` and a compatible Rust
toolchain. It installs the pinned license tool and downloads any uncached
Rust dependencies, so network access may be needed. It does not build or
install the SDK or the example extension.

The command installs the license tool into ``CARGO_INSTALL_ROOT`` when set,
otherwise ``CARGO_HOME`` or ``~/.cargo``. It passes that location explicitly
to Cargo and adds its ``bin`` directory to the license generator's ``PATH``.
No manual ``PATH`` change is needed for the refresh.

Without ``--refresh-example``, sync updates version metadata and manifests,
but leaves the lockfile and license report unchanged and does not require
Rust. ``--skip-example-updates`` is for SDK artifact-only builds; it cannot be
combined with ``--refresh-example`` and must not be used for a version update.

.. _bsk-sdk-task-rust-dependencies:

Review the Refreshed Example
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The sync commands above complete the Rust refresh automatically, in this order:

#. Update the example's Python requirements and Rust manifests from the
   selected Basilisk source.
#. Run ``cargo generate-lockfile`` to resolve that dependency graph.
#. Run ``cargo fetch --locked`` to populate the cache for the license scan.
#. Install the ``cargo-about`` version recorded in the newly synchronized
   ``src/bsk_sdk/rust/support-versions.json``.
#. Regenerate ``custom_atm/RUST-THIRD-PARTY.txt`` with the synchronized
   license generator and ``--require-tool``. A missing or incorrect tool
   version fails the command instead of silently skipping the report.

Review the example files in the version-update table before packaging.
Changing between local paths and tags, or changing upstream dependencies,
can change both the lockfile and license report. A refresh can also select
newer compatible third-party dependencies. No additional refresh commands
or manual edits to synchronized fields are needed.

These steps prepare the bundled Rust example; downstream extensions using
only C/C++ do not need Rust.

.. _bsk-sdk-task-install-basilisk:

Install the Matching Basilisk Package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Choose one method below. For a nightly, install the wheel first and use its
build metadata to select the source. For a release or local build, install
the package corresponding to the source already selected.

.. _bsk-sdk-install-final:

Final Release from PyPI
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   python -m pip install --force-reinstall "bsk[all]==${BSK_VERSION}"

.. _bsk-sdk-install-candidate:

Release Candidate from TestPyPI
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   python -m pip install --pre --force-reinstall \
     --index-url https://test.pypi.org/simple/ \
     --extra-index-url https://pypi.org/simple/ \
     "bsk[all]==${BSK_VERSION}"

.. _bsk-sdk-install-nightly:

Latest Develop Nightly and Matching Source
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

With ``BSK_ROOT`` set to the checkout intended for this validation:

.. code-block:: bash

   python -m pip install --pre --upgrade --force-reinstall --no-cache-dir \
     --index-url https://avslab.github.io/basilisk/nightly/ \
     --extra-index-url https://pypi.org/simple/ \
     --report "$CHECK_DIR/bsk-nightly-install.json" \
     "bsk[all]"

Check that pip obtained ``bsk`` from the nightly index. Pip also considers
packages on the extra index, so the index arguments alone do not establish
where the selected wheel came from:

.. code-block:: bash

   python - "$CHECK_DIR/bsk-nightly-install.json" <<'PY'
   import json
   import sys
   from pathlib import Path
   report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
   urls = [item["download_info"]["url"] for item in report["install"]
           if item["metadata"]["name"].lower() == "bsk"]
   assert len(urls) == 1 and urls[0].startswith(
       "https://avslab.github.io/basilisk/nightly/bsk/"
   ), f"Expected a BSK nightly wheel, got {urls}"
   PY
   BSK_REVISION="$(python - <<'PY'
   import Basilisk
   artifact = Basilisk.getBuildInfo()["artifact"]
   assert artifact["sourceDirty"] is False
   print(artifact["sourceRevision"], end="")
   PY
   )"
   git -C "$BSK_ROOT" fetch origin "$BSK_REVISION"
   git -C "$BSK_ROOT" checkout --detach FETCH_HEAD

This selects the develop commit used to build the wheel, which may lag the
current tip of ``develop``. To test a newer commit that has no matching wheel
yet, build Basilisk locally from that commit.

.. _bsk-sdk-install-local:

Local Beta or Feature Branch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build a Basilisk wheel from the selected clean checkout and install it into
the active SDK test environment:

.. code-block:: bash

   CONAN_ARGS="--clean" python -m pip wheel --no-deps -v \
     -w "$CHECK_DIR/basilisk" "$BSK_ROOT"
   python -m pip install --force-reinstall "$CHECK_DIR"/basilisk/bsk-*.whl

If the extension requires optional Basilisk components such as OpNav, also
build the required optional components from that same source using Basilisk's
build instructions.

.. _bsk-sdk-task-build-sdk:

Build and Test the SDK Wheel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   BSK_SDK_AUTO_SYNC=0 python -m build --wheel --outdir "$CHECK_DIR/sdk"
   python -m pip install --force-reinstall "$CHECK_DIR"/sdk/bsk_sdk-*.whl
   python -m pytest tests -v

The explicit sync has already selected and copied the source. Keep automatic
sync disabled for this build, especially when testing an unrecorded checkout.

.. _bsk-sdk-task-verify:

Verify Versions and Source Revisions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

After installing both wheels, compare the source, SDK, and installed Basilisk
versions, then check the wheel's source revision:

.. code-block:: bash

   python - "$BSK_ROOT" <<'PY'
   import subprocess
   import sys
   from pathlib import Path
   import Basilisk
   import bsk_sdk
   source = Path(sys.argv[1])
   expected = (source / "docs/source/bskVersion.txt").read_text().strip()
   assert bsk_sdk.__version__ == bsk_sdk.bsk_version() == expected
   assert Basilisk.__version__ == expected
   artifact = Basilisk.getBuildInfo()["artifact"]
   revision = subprocess.check_output(
       ["git", "-C", str(source), "rev-parse", "HEAD"], text=True
   ).strip()
   assert artifact["sourceDirty"] is False
   assert artifact["sourceRevision"] == revision
   print(f"BSK and BSK-SDK: {expected}; Basilisk source: {revision}")
   PY

For publication, also verify the expected Basilisk tag and review both Cargo
manifests for ``tag = "v<BSK_VERSION>"``. Development-mode manifests should
instead contain local paths to the selected checkout. The synchronized
``pyproject.toml`` and example requirements belong in the Git review as well.

.. _bsk-sdk-task-test-extension:

Build and Test the Example Extension
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   python -m build --wheel --no-isolation \
     --outdir "$CHECK_DIR/extension" \
     -Cbuild-dir="$CHECK_DIR/extension-build" examples/custom-atm-extension
   python -m pip install --force-reinstall --no-deps \
     "$CHECK_DIR"/extension/*.whl
   python -c "import Basilisk, numba, custom_atm; from custom_atm import customExponentialAtmosphere, numbaAtmosphere, rustAtmosphere"
   python -m pytest examples -v

``--no-isolation`` uses the SDK wheel just installed. ``--no-deps`` preserves
the selected Basilisk wheel; the environment task installs Numba for this
example. The import check ensures a missing Basilisk installation cannot
turn the example tests into skipped tests that appear successful.

.. _bsk-sdk-task-commit:

Review and Commit the Preparation Files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Inspect both unstaged changes and the submodule pointer already staged during
source selection:

.. code-block:: bash

   git diff --submodule=log
   git diff --cached --submodule=log
   git status --short

For a normal committed beta or release update, review and stage these files:

.. code-block:: bash

   git add external/basilisk pyproject.toml src/bsk_sdk/_bsk_version.txt \
     examples/custom-atm-extension/pyproject.toml \
     examples/custom-atm-extension/Cargo.toml \
     examples/custom-atm-extension/rustAtmosphere/Cargo.toml \
     examples/custom-atm-extension/Cargo.lock \
     examples/custom-atm-extension/custom_atm/RUST-THIRD-PARTY.txt

Some entries may be unchanged. Keep commits focused, with messages identifying
the cycle or release and any Rust dependency transition. Include additional
SDK fixes or documentation deliberately; do not stage ignored generated
payload directories. A separate local Basilisk checkout is useful for
exploration, but does not update the submodule record that ordinary release
CI uses.

CI and Publishing Behavior
--------------------------

* **Beta CI:** ``CI`` uses the version in ``src/bsk_sdk/_bsk_version.txt``
  (or a manual override) to select the nightly channel for alpha/beta builds.
  It installs the latest nightly, checks its download origin, and syncs from
  the wheel's exact source revision. Example and MSRV jobs retain that channel
  and verify their source against the revision used to build the SDK wheel.
  They use ``--refresh-example`` to prepare the example for that source.
* **Scheduled nightly:** tests the latest SDK ``develop`` against the latest
  available BSK develop wheel and its matching source. Nightly jobs pass
  ``--local-rust-dependencies --refresh-example`` even if the wheel reports
  an RC/final version.
  A manual ``Nightly`` run uses the selected SDK branch.
* **Final/candidate CI:** ordinary validation uses the recorded submodule and
  the matching PyPI/TestPyPI package. A manual version override selects the
  corresponding Basilisk tag instead and refreshes the example for it.
* **Publication:** pushing ``v2.X.Y`` starts ``Publish Wheels`` for PyPI;
  ``v2.X.YrcN`` routes to TestPyPI. Publication clones the Basilisk tag,
  synchronizes the SDK, checks the committed Rust manifests, lockfile, and
  license report, and builds/tests the Rust example before uploading the
  SDK wheel and source distribution. It does not use ``--refresh-example``:
  stale committed files must fail validation. Alpha/beta publication is
  rejected.

If a newer nightly appears between the SDK build and the extension jobs, the
source-revision check fails explicitly. Rerun the whole workflow to rebuild
the SDK and test it against the same nightly. No temporary workflow edit is
needed for normal beta, major, or patch updates.
