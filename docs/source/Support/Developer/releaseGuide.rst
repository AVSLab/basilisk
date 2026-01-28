
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
#. Ensure documentation builds without warnings or errors.
#. Push branch to origin and do a PR.
#. Merge ``develop`` into ``master``
#. Create a Release on GitHub

To prepare ``develop`` for the next beta cycle:

#. Create ``beta_X_Y`` branch
#. Modify the version number in ``docs/source/bskVersion.txt``
   to reflect the new beta cycle such as ``v2_X_Y_beta``.
#. Update ``docs/source/Support/bskReleaseNotes.rst``
   and ``docs/source/Support/bskKnownIssues.rst`` files to next beta cycle
#. Make PR and push to ``develop``


The following documentation provides detailed instructions for some of these steps.

Updating the Version Number
---------------------------
The version number is stored in the ``docs/source/bskVersion.txt`` file. This file
gets read by the ``pyproject.toml`` during packaging to set the version of the
package. Previously, this file was updated automatically during the CI/CD
process; however, it must now be updated manually to prevent excessive version
bumps.

Update this file to reflect the new version number following semantic versioning
guidelines (e.g., MAJOR.MINOR.PATCH).

Updating the Changelog
----------------------
The changelog is located in the ``docs/source/Support/bskReleaseNotes.rst`` file.
Before releasing a new version, ensure that this file is updated to include
all relevant changes, bug fixes, and new features that are part of the release.
Add a release date after the version number.

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
Releases are published by pushing a git tag to the ``master`` branch. Our GitHub Actions workflow
(``Publish Wheels``) is triggered on tag pushes matching:

- ``v[0-9]*``  (real releases, published to PyPI)

This means: **pushing a tag kicks off the wheel builds on all supported
platforms, builds an sdist, and then publishes the artifacts**.

Tag format
^^^^^^^^^^
Release tags must follow the format ``vX.Y.Z`` (for example, ``v2.9.0``).

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
#. Select correct commit on ``master``
#. Add title “Basilisk v2.x.x”
#. Add release notes
#. Remove all ``:ref:`` statements from release notes
#. Add release tag ``v2.X.Y``.
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
