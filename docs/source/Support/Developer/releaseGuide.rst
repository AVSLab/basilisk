
.. _releaseGuide:

Release Guide
=============

These docs outline the process for releasing a new version of Basilisk. It is
intended for developers and maintainers of the Basilisk software.

The high level steps to release a new version of Basilisk are as follows:

#. Create a new branch ``v2_X_Y`` on ``develop`` for the release.
#. Modify the version number in ``docs/source/bskVersion.txt``
   to reflect the new release.
#. Add release date to ``docs/source/Support/bskReleaseNotes.rst``
   and ``docs/source/Support/bskKnownIssues.rst`` files.
#. Integrate content of ``docs/source/Support/bskReleaseNotesSnippets/_compiled_latest.rst`` into ``bskReleaseNotes.rst``.
   Delete the release notes snippets.
#. Ensure documentation builds without warnings or errors.
#. Push branch to origin and do a PR.
#. Merge ``develop`` into ``master``
#. Add release tag ``v2.X.Y`` to ``master``
#. Create a Release on GitHub

To prepare ``develop`` for the next beta cycle:

#. Create ``beta_X_Y`` branch
#. Modify the version number in ``docs/source/bskVersion.txt``
   to reflect the new beta cycle such as ``v2_X_Y_beta``.
#. Update ``docs/source/Support/bskReleaseNotes.rst``
   and ``docs/source/Support/bskKnownIssues.rst`` files to next beta cycle
#. Make PR and push to ``develop``
#. Manually run the ``Nightly Wheels`` workflow using the ``workflow_dispatch``
   trigger after merging the beta branch so the nightly package index is
   republished on GitHub Pages.


The following documentation provides detailed instructions for some of these steps.

Updating the Version Number
---------------------------
The version number is stored in the ``docs/source/bskVersion.txt`` file. This file
gets read by the ``pyproject.toml`` during packaging to set the version of the
package. Previously, this file was updated automatically during the CI/CD
process; however, it must now be updated manually to prevent excessive version
bumps.

Update this file to reflect the new version number following semantic versioning
guidelines (e.g., ``MAJOR.MINOR.PATCH``).

For a release candidate, set the version to ``MAJOR.MINOR.PATCHrcN``
(e.g., ``2.10.2rc1``). This ensures artifacts built from an RC tag are
correctly identified as pre-release and are published to TestPyPI rather than
production PyPI. Once the release candidate is validated and a final release tag
is pushed, reset ``bskVersion.txt`` to the plain ``MAJOR.MINOR.PATCH`` form
(e.g., ``2.10.2``) before tagging.

Updating the Release Notes
--------------------------
For normal PR development, release notes are contributed as snippet files in:
``docs/source/Support/bskReleaseNotesSnippets``.
These snippets are compiled and included automatically into the current
``Version |release|`` section in ``docs/source/Support/bskReleaseNotes.rst`` when documentation is built.

Creating a Release Branch
-------------------------
Active development occurs on the ``develop`` branch. Once all commits for a new
minor release have been merged into ``develop``, and the version number and
changelog have been updated, a new release branch should be created.

The release branch is named after the minor version (e.g. ``2.9``) and is a
long-lived maintenance branch. It represents the released ``X.Y`` line and is
used for the initial release as well as any future patch releases (``X.Y.1``,
``X.Y.2``, etc.).

All patches and fixes for the ``X.Y`` release line must be made on this branch.
Any changes applied to the release branch should also be merged or cherry-picked
back into ``develop`` to ensure they are included in future releases.

Creating and Pushing Tags
-------------------------
Releases are published by pushing a git tag. The ``Publish Wheels`` GitHub Actions
workflow triggers on tag pushes and routes artifacts based on the tag name:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Tag pattern
     - Destination
   * - ``vX.Y.Z`` (no ``rc``)
     - PyPI (public release)
   * - ``vX.Y.ZrcN``
     - TestPyPI (release candidate)
   * - ``test*``
     - TestPyPI (ad-hoc testing)

This means: **pushing a tag kicks off the wheel builds on all supported
platforms, builds an sdist, and then publishes the artifacts**.

Tag format
^^^^^^^^^^
- Full release tags must follow the format ``vX.Y.Z`` (for example, ``v2.9.0``).
- Release candidate tags follow ``vX.Y.ZrcN`` (for example, ``v2.10.2rc1``).

When to use a release candidate tag
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Push an RC tag when you want to publish a pre-release build to TestPyPI for
validation before committing to a final release. Typical scenarios include:

- Verifying that the wheel build pipeline works end-to-end for a new version.
- Giving early adopters a chance to test a release before it is published to
  production PyPI.
- Debugging the publish workflow itself without touching the production index.

The workflow for an RC release is:

#. Set ``docs/source/bskVersion.txt`` to the RC version (e.g. ``2.10.2rc1``).
#. Push an RC tag (e.g. ``v2.10.2rc1``) — CI builds wheels and publishes to
   TestPyPI automatically.
#. Validate the artifacts from TestPyPI.
#. If further changes are needed, increment the RC counter (``rc2``, ``rc3``,
   …) and repeat.
#. Once the RC is validated, reset ``bskVersion.txt`` to the final version
   (e.g. ``2.10.2``) and push the release tag (e.g. ``v2.10.2``) to trigger
   the production PyPI publish.

Where to tag
^^^^^^^^^^^^
Tags for the ``X.Y`` patch release line must be created from the corresponding
maintenance branch (e.g. create ``v2.9.1`` from branch ``2.9``).

Notes
^^^^^
- Tag pushes are immutable release triggers: once a tag is pushed, CI will build
  and attempt to publish. If a mistake is made, prefer bumping the version and
  tagging again rather than reusing the same version tag.
- After patching on ``X.Y``, merge or cherry-pick the fix back into ``develop``
  so the next release line also contains the change.

Creating a Release on GitHub
----------------------------
After pushing the release tag, create a new release on GitHub:

#. Go to “Code” tab, on right column select Releases
#. Select “Draft new Release”
#. Select correct tag ``v2.X.Y`` on ``master``
#. Add title “Basilisk v2.X.Y”
#. Add release notes
#. Remove all ``:ref:`` statements from release notes
#. Bottom of page, press “Publish Release” button

This will make the release official and provide users with information about the new version.

Patch and Backport Workflow
---------------------------
Once a release branch (``X.Y``) exists we only maintain this latest ``X.Y`` release on ``master``
and the current beta on ``develop``.

Where to make changes
^^^^^^^^^^^^^^^^^^^^^
- **New features or breaking changes**:
  Go directly into ``develop`` for the next beta cycle (never into a release branch).

- **Bug fixes for an existing release**:

   #. Create branch on develop to test the fix.
   #. Create PR for this fix branch, don't update beta release notes
   #. On master, create a patch branch and cherry pick over the fix commit(s)
   #. On the patch branch, update the release notes for ``v2.X.Y`` with a release date
   #. Merge this branch into ``master``
   #. Create a Release on GitHub
   #. Charry pick the commit that updated the ``v2.X.Y`` release notes back into
      the beta cycle on ``develop``.


.. important::

   - Never develop directly on multiple branches in parallel for the same fix.
     This leads to divergence and hard-to-resolve conflicts.
   - Do not merge ``develop`` back into ``X.Y`` after the release branch is
     created. Release branches must remain stable and focused on fixes only.
   - Cherry-picks should be small, focused, and preferably reference the original
     commit hash in the message.

Releasing the BSK SDK
---------------------
The `bsk-sdk <https://github.com/AVSLab/bsk_sdk>`_ package vendors the Basilisk
SDK headers and runtime for plugin authors. Its version is kept in sync with
Basilisk, so a new BSK release requires a corresponding SDK release.

CI automatically checks out the Basilisk submodule at the tag matching the
version string in ``pyproject.toml``, so no manual submodule update is needed.
The only manual steps are:

#. On a branch of develop, update the ``version`` field in ``pyproject.toml``
   to match the new BSK release (e.g. ``2.X.Y``).
#. Open a PR merging the updated ``pyproject.toml`` into ``develop`` on the
   ``bsk-sdk`` repo. The PR CI workflow will test the SDK wheel build.
#. Merge ``develop`` into ``master``.
#. Push the matching tag ``v2.X.Y`` to ``master`` to trigger the wheel build
   and PyPI publish via GitHub Actions.
