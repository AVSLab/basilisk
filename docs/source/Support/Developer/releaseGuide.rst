
.. _releaseGuide:

Release Guide

=================

These docs outline the process for releasing a new version of Basilisk. It is
intended for developers and maintainers of the Basilisk software.

The high level steps to release a new version of Basilisk are as follows:

1. **Update Version Number**: Modify the version number in `docs/source/bskVersion.txt`
    to reflect the new release.
2. **Update Changelog**: Edit the release notes in `docs/source/Support/bskReleaseNotes.rst` file to include the changes.
3. **Create a Release Branch**: Create a new branch from the main development branch for the release.
4. **Create a Tag**: Once the release branch is ready, create a git tag for the new version.
5. **Create a release on GitHub**: Go to the GitHub repository and create a new release using the tag created in the previous step.


The following documentation provides detailed instructions for each of these steps.

==========================
Updating the Version Number
===========================

The version number is stored in the `docs/source/bskVersion.txt` file. This file
gets read by the `pyproject.toml` during packaging to set the version of the
package. Previously, this file was updated automatically during the CI/CD
process; however, it must now be updated manually to prevent excessive version
bumps.

Update this file to reflect the new version number following semantic versioning
guidelines (e.g., MAJOR.MINOR.PATCH).

======================
Updating the Changelog
======================

The changelog is located in the `docs/source/Support/bskReleaseNotes.rst` file.
Before releasing a new version, ensure that this file is updated to include
all relevant changes, bug fixes, and new features that are part of the release.

=========================
Creating a Release Branch
=========================

Active development occurs on the ``develop`` branch. Once all commits for a new
minor release have been merged into ``develop``, and the version number and
changelog have been updated, a new release branch should be created.

The release branch is named after the minor version (e.g. ``3.9``) and is a
long-lived maintenance branch. It represents the released ``X.Y`` line and is
used for the initial release as well as any future patch releases (``X.Y.1``,
``X.Y.2``, etc.).

To create a release branch:

.. code-block:: bash

    git checkout develop
    git pull origin develop
    git checkout -b X.Y
    git push -u origin X.Y

Replace ``X.Y`` with the new minor version number (for example, ``3.9``).

All patches and fixes for the ``X.Y`` release line must be made on this branch.
Any changes applied to the release branch should also be merged or cherry-picked
back into ``develop`` to ensure they are included in future releases.

=========================
Creating and Pushing Tags
=========================

Releases are published by pushing a git tag. Our GitHub Actions workflow
(``Publish Wheels``) is triggered on tag pushes matching:

- ``v[0-9]*``  (real releases, published to PyPI)

This means: **pushing a tag kicks off the wheel builds on all supported
platforms, builds an sdist, and then publishes the artifacts**.

Tag format
----------

Release tags must follow the format ``vX.Y.Z`` (for example, ``v3.9.0``).

Where to tag
------------

Tags for the ``X.Y`` release line must be created from the corresponding
maintenance branch (e.g. create ``v3.9.1`` from branch ``3.9``).

Creating and pushing a release tag (publishes to PyPI)
------------------------------------------------------

.. code-block:: bash

    git checkout X.Y
    git pull origin X.Y
    git tag -a vX.Y.Z -m "Release vX.Y.Z"
    git push origin vX.Y.Z

Replace ``X.Y`` with the release branch name (e.g. ``3.9``) and ``X.Y.Z`` with
the full version (e.g. ``3.9.0``).

Notes
-----

- Tag pushes are immutable release triggers: once a tag is pushed, CI will build
  and attempt to publish. If a mistake is made, prefer bumping the version and
  tagging again rather than reusing the same version tag.
- After patching on ``X.Y``, merge or cherry-pick the fix back into ``develop``
  so the next release line also contains the change.

=============================
Creating a Release on GitHub
=============================

After pushing the release tag, create a new release on GitHub:

1. Navigate to the GitHub repository for Basilisk.
2. Click on the "Releases" tab.
3. Click on "Draft a new release".
4. In the "Tag version" dropdown, select the tag you just pushed (e.g. ``vX.Y.Z``).
5. Fill in the release title and description, summarizing the changes included in this release.
6. Click "Publish release".

This will make the release official and provide users with information about the new version.

===========================
Patch and Backport Workflow
===========================

Once a release branch (``X.Y``) exists, there may be multiple active branches at
the same time (e.g. ``develop``, ``3.9``, ``3.8``). To keep fixes consistent
across versions, we follow a strict patch and backport policy.

General rule
------------

- **All fixes must be applied to every supported branch that needs them.**
- The fix should be authored on the *oldest applicable release branch* and then
  cherry-picked forward.

Where to make changes
---------------------

- **Bug fixes for an existing release**:
  Start from the corresponding release branch (e.g. ``3.9``).

- **New features or breaking changes**:
  Go directly into ``develop`` (never into a release branch).

Applying a patch release fix
----------------------------

Example: fixing a bug that affects the ``3.9`` release line.

1. Create the fix on the ``3.9`` branch:

.. code-block:: bash

    git checkout 3.9
    git pull origin 3.9
    git checkout -b fix/some-bug
    # implement fix
    git commit

2. Merge the fix into ``3.9`` (via PR or direct merge).

3. Cherry-pick the same commit(s) onto ``develop``:

.. code-block:: bash

    git checkout develop
    git pull origin develop
    git cherry-pick <commit-sha>

If multiple release branches are active (e.g. ``3.8`` and ``3.9``), repeat the
cherry-pick for each newer branch as needed.

Important notes
---------------

- **Never develop directly on multiple branches in parallel** for the same fix.
  This leads to divergence and hard-to-resolve conflicts.
- **Do not merge ``develop`` back into ``X.Y``** after the release branch is
  created. Release branches must remain stable and focused on fixes only.
- Cherry-picks should be small, focused, and preferably reference the original
  commit hash in the message.

After the fix has been merged into the release branch, create a new patch tag
(e.g. ``v3.9.1``) to publish the updated release.
