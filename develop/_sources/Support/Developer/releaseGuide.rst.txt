
.. _releaseGuide:

Release Guide
=============

Major Release Guide
-------------------

These docs outline the process for releasing a new version of Basilisk. It is
intended for developers and maintainers of the Basilisk software.

The high level steps to release a new version of Basilisk are as follows:

#. Create a new branch ``feature/v2_X_Y`` on ``develop`` for the release.
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
#. When BSK wheels are successfully deployed, create a Release on GitHub

To prepare ``develop`` for the next beta cycle:

#. Create ``beta_X_Y`` branch
#. Modify the version number in ``docs/source/bskVersion.txt``
   to reflect the new beta cycle such as ``v2.X.Yb0``.
#. Update ``docs/source/Support/bskReleaseNotes.rst``
   and ``docs/source/Support/bskKnownIssues.rst`` files to next beta cycle
#. Make PR and push to ``develop``
#. Manually run the ``Nightly Wheels`` workflow using the ``workflow_dispatch``
   trigger after merging the beta branch so the nightly wheels are
   republished on GitHub Pages.


The following documentation provides detailed instructions for some of these steps.

Updating the Version Number
^^^^^^^^^^^^^^^^^^^^^^^^^^^
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
^^^^^^^^^^^^^^^^^^^^^^^^^^
For normal PR development, release notes are contributed as snippet files in:
``docs/source/Support/bskReleaseNotesSnippets``.
These snippets are compiled and included automatically into the current
``Version |release|`` section in ``docs/source/Support/bskReleaseNotes.rst`` when documentation is built.

Creating a Release Branch
^^^^^^^^^^^^^^^^^^^^^^^^^
Active development occurs on the ``develop`` branch. Once all commits for a new
minor release have been merged into ``develop``, then a release branch ``feature/v2_X_0`` is created
off of ``develop`` where the version number and changelog are updated.  After ``feature/v2_X_0``
is merged into ``develop``, ``develop`` is merged into ``master`` and that commit is tagged with ``v2.X.0``.

All patches and fixes for the ``2.X`` release line must be made on the ``patch/v2_X_x`` branch.
Any changes applied to the release branch should also be merged or cherry-picked
back into ``develop`` to ensure they are included in future releases.

Creating and Pushing Tags
^^^^^^^^^^^^^^^^^^^^^^^^^
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
""""""""""
- Full release tags must follow the format ``vX.Y.Z`` (for example, ``v2.9.0``).
- Release candidate tags follow ``vX.Y.ZrcN`` (for example, ``v2.10.2rc1``).

When to use a release candidate tag
"""""""""""""""""""""""""""""""""""
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
""""""""""""
Tags for the ``2.X.Y`` patch release line must be created from the corresponding
maintenance branch (e.g. create tag ``v2.9.1`` on the branch ``patch/v2_9_x``).

Notes
"""""
- Tag pushes are immutable release triggers: once a tag is pushed, CI will build
  and attempt to publish. If a mistake is made, prefer bumping the version and
  tagging again rather than reusing the same version tag.
- After patching on ``v2.X``, merge or cherry-pick the fix back into ``develop``
  so the next release line also contains the change.

Creating a Release on GitHub
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
After pushing the release tag, create a new release on GitHub:

#. Go to “Code” tab, on right column select Releases
#. Select “Draft new Release”
#. Select correct tag ``v2.X.Y`` on ``master``
#. Add title “Basilisk v2.X.Y”
#. Add release notes manually or using the auto-generation option
#. Remove all ``:ref:`` statements, if needed, from release notes
#. Bottom of page, press “Publish Release” button

This will make the release official and provide users with information about the new version.

Patch Release Guide
-------------------
Once a major tagged release (``v2.X.0``) exists we only maintain this latest ``v2.X.Y`` patch release on
a branch ``patch/v2_X_x`` and the current beta on ``develop``.

To release a patch version ``v2.X.Y`` the following steps are used:

#. If this is the first patch since ``v2.X``, create a branch off the ``v2.X.0`` tagged release on ``master``
   and call it ``patch/v2_X_x``.  Pull this ``patch/v2_X_x`` branch.
#. Cherry pick over the commits from ``develop`` for this patch release.
#. Update Basilisk version number in ``docs/source/bskVersion.txt`` to ``2.X.Y``.
#. Update ``docs/source/Support/bskReleaseNotes.rst`` by
   removing release notes snippets to add them to ``bskReleaseNotes.rst`` directly in a dated section for ``2.X.Y``.
#. If needed, update ``docs/source/Support/bskKnownIssues.rst`` to have a section for ``2.X.Y``.
#. Locally test a clean build of Basilisk and documentation.
#. Push branch to ``origin``.
#. On https://github.com/AVSLab/basilisk/actions, run the action "Pull Request" on the branch ``patch/v2_X_x``.
   Wait for tests to complete.
#. Add a tag ``v2.X.Y`` to this branch and push tag to origin.  This will trigger the wheel build process.
#. The last step does not trigger the patch release documentation process as this patch release is neither
   on ``master`` or ``develop``.  On https://github.com/AVSLab/basilisk/actions manually run the
   action ``Merge to master or develop`` on the branch ``patch/v2_X_x``.
#. The last step deletes the online developer documentation and it needs to be rebuilt.
   On https://github.com/AVSLab/basilisk/actions manually run the
   action ``Merge to master or develop`` on the branch ``develop``.
#. Create a Release on GitHub

.. important::

   - Never develop directly on multiple branches in parallel for the same fix.
     This leads to divergence and hard-to-resolve conflicts.
   - Do not merge ``develop`` back into ``v2.X`` after the release branch is
     created. Release branches must remain stable and focused on fixes only.
   - Cherry-picks should be small, focused, and preferably reference the original
     commit hash in the message.

Releasing the BSK SDK
---------------------
The `bsk-sdk <https://github.com/AVSLab/bsk_sdk>`_ package vendors the Basilisk
SDK headers and runtime for plugin authors. Its version is kept in sync with
Basilisk, so a new BSK release requires a corresponding SDK release.

The following steps are to be done after the Basilisk ``v2.X.Y`` release is fully completed.
This ensures that the Basilisk release is completed and available on the expected Basilisk branch.

Major SDK Release
^^^^^^^^^^^^^^^^^
To release a major version ``2.X.0`` the following steps are used:

#. On a branch of develop, update the ``version`` field in ``pyproject.toml``
   to match the new major BSK release (e.g. ``2.X.0``).
#. Push branch to ``origin`` and create a PR to trigger the CI workflow to test the SDK wheel.
#. Merge the PR to ``develop`` once CI tests pass.
#. Merge ``develop`` into ``master``.
#. Add the tag ``v2.X.0`` to ``master`` and push tag to origin to trigger the wheel build
   and PyPI publish via GitHub Actions.
#. Create a Release on GitHub

Patch Release
^^^^^^^^^^^^^
To release a patch version ``2.X.Y`` the following steps are used:

#. If this is the first patch since ``v2.X``, create a branch off the ``v2.X.0`` tagged release on ``master``
   and call it ``patch/v2_X_x``.  Pull this ``patch/v2_X_x`` branch.
#. Cherry pick over the commits from ``develop`` for this patch release.
#. Update the ``version`` field in ``pyproject.toml`` to match the patch BSK release (e.g. ``2.X.Y``).
#. Update ``.github/workflows/ci.yml``, section ``Install bsk-sdk wheel and Basilisk``, to pull the matching
   Basilisk branch ``--branch patch/v2_X_x``
#. Commit edits and push to ``origin``
#. Manually run the ``CI`` action on https://github.com/AVSLab/bsk_sdk/actions. Wait for tests to complete.
#. Add the tag ``v2.X.Y`` to ``patch/v2_X_x`` and push tag to origin to trigger the wheel build
   and PyPI publish via GitHub Actions.
#. Create a Release on GitHub
