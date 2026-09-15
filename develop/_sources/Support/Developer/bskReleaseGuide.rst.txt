.. _releaseGuide:
.. _bskReleaseGuide:

BSK Release Guide
=================

This guide describes how Basilisk maintainers prepare, validate, and publish
major and patch releases. In this guide, a *major Basilisk release* means a
scheduled ``2.X.0`` feature release, while a *patch release* means ``2.X.Y`` for
``Y > 0``.

The release workflows contain the ordering and branch decisions. Detailed
commands and procedures are collected under :ref:`Common Release Tasks
<bsk-release-common-tasks>` so the same instructions can be used for both
major and patch releases.

Release Model
-------------

The version in ``docs/source/bskVersion.txt``, the release tag, and the commit
being published must agree. Package versions use PEP 440 syntax without a
leading ``v``; Git tags add the leading ``v``.

.. list-table:: Basilisk release types
   :header-rows: 1
   :widths: 20 25 20 20 15

   * - Release type
     - Source branch
     - Package version
     - Tag location
     - Destination
   * - Major
     - ``develop`` through ``feature/v2_X_0``
     - ``2.X.0``
     - ``master``
     - PyPI
   * - Patch
     - ``patch/v2_X_x``
     - ``2.X.Y``
     - Patch branch
     - PyPI
   * - Release candidate
     - Branch being validated
     - ``2.X.YrcN``
     - Tested commit
     - TestPyPI

Pushing a version tag starts the ``Publish Wheels`` GitHub Actions workflow.
Final tags publish to PyPI, while release-candidate and ``test*`` tags publish
to TestPyPI.

Release Workflows
-----------------

.. _bsk-major-release:

Major Release
~~~~~~~~~~~~~

Use this workflow for a scheduled ``2.X.0`` release. After publication, finish
the workflow by starting the next beta cycle on ``develop``.

Prepare
^^^^^^^

#. Create ``feature/v2_X_0`` from ``develop`` after all intended release
   content has been merged.
#. :ref:`Finalize the release notes and known issues
   <bsk-release-task-notes>` for ``2.X.0``.
#. If a release candidate is required, follow :ref:`Validate a Release
   Candidate <bsk-release-task-rc>` on the release branch.
#. :ref:`Set and verify the final version <bsk-release-task-version>` as
   ``2.X.0``.

Validate
^^^^^^^^

#. :ref:`Run clean local validation <bsk-release-task-local-validation>`.
#. Push the release branch and open a PR to ``develop``.
#. Wait for the ``Pull Request`` workflow to pass before merging.

Publish
^^^^^^^

#. Merge the release PR into ``develop``.
#. Merge ``develop`` into ``master``.
#. :ref:`Create and push the final tag <bsk-release-task-tag>` ``v2.X.0``
   from ``master``.
#. Wait for the ``Publish Wheels`` workflow to finish and verify the published
   artifacts.
#. :ref:`Create the GitHub Release <bsk-release-task-github-release>`.

Start the Next Beta Cycle
^^^^^^^^^^^^^^^^^^^^^^^^^

After the major release is published, prepare ``develop`` for the next major
release:

#. Create a ``beta_X_Y`` branch from ``develop`` for the next planned release.
#. Set ``docs/source/bskVersion.txt`` to the next beta version, such as
   ``2.X.0b0``. Do not include a leading ``v``.
#. In ``bskReleaseNotes.rst``, change the completed release heading to its
   literal version, then create a new ``Version |release|`` section above it
   containing the active ``_compiled_latest.rst`` include.
#. In ``bskKnownIssues.rst``, change the completed release heading to its
   literal version and create a new ``Version |release|`` section above it for
   the new beta cycle.
#. Open a PR from the beta branch to ``develop`` and wait for CI to pass before
   merging.
#. After merging, manually run the ``Nightly Wheels`` workflow on ``develop``
   using ``workflow_dispatch``. Verify that the nightly package index on GitHub
   Pages contains the new beta version.

.. _bsk-patch-release:

Patch Release
~~~~~~~~~~~~~

Use this workflow for ``2.X.Y`` after ``v2.X.0`` has been published. Only the
latest ``2.X.Y`` line is maintained on ``patch/v2_X_x``; current development
continues independently on ``develop``.

Prepare
^^^^^^^

#. For the first patch in the release line, create ``patch/v2_X_x`` from the
   ``v2.X.0`` tag. For later patches, use the existing patch branch based on
   the latest ``v2.X.Y`` release.
#. Cherry-pick the approved fixes from ``develop``. Keep each cherry-pick
   focused and retain the original commit reference.
#. :ref:`Finalize the release notes and known issues
   <bsk-release-task-notes>` for ``2.X.Y``.
#. If a release candidate is required, follow :ref:`Validate a Release
   Candidate <bsk-release-task-rc>` on the patch branch.
#. :ref:`Set and verify the final version <bsk-release-task-version>` as
   ``2.X.Y``.

Validate
^^^^^^^^

#. :ref:`Run clean local validation <bsk-release-task-local-validation>`.
#. Push ``patch/v2_X_x`` to ``origin``.
#. Manually run the ``Pull Request`` workflow on ``patch/v2_X_x`` and wait for
   all jobs to pass.

Publish
^^^^^^^

#. :ref:`Create and push the final tag <bsk-release-task-tag>` ``v2.X.Y``
   from ``patch/v2_X_x``.
#. Wait for the ``Publish Wheels`` workflow to finish and verify the published
   artifacts.
#. :ref:`Deploy the patch documentation <bsk-release-task-patch-docs>`.
#. :ref:`Create the GitHub Release <bsk-release-task-github-release>`.

Close Out
^^^^^^^^^

#. Confirm that every patch fix also exists on ``develop``. If a fix originated
   on the patch branch, :ref:`forward-port it <bsk-release-task-forward-port>`.
#. Keep ``patch/v2_X_x`` available for subsequent patches in the same release
   line.

.. _bsk-release-common-tasks:

Common Release Tasks
--------------------

.. _bsk-release-task-version:

Set and Verify the Version
~~~~~~~~~~~~~~~~~~~~~~~~~~

The package version is stored in ``docs/source/bskVersion.txt`` and read by
``pyproject.toml`` during packaging. Update it manually and use PEP 440 syntax:

.. list-table:: Version and tag examples
   :header-rows: 1
   :widths: 35 30 35

   * - Build type
     - Package version
     - Git tag
   * - Beta
     - ``2.X.0bN``
     - No release tag
   * - Release candidate
     - ``2.X.YrcN``
     - ``v2.X.YrcN``
   * - Final
     - ``2.X.Y``
     - ``v2.X.Y``

Before tagging, confirm that the version file, intended tag, and checked-out
commit all describe the same release. Commit every version change before
creating its tag.

.. _bsk-release-task-notes:

Finalize the Release Notes and Known Issues
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Normal PRs contribute release-note snippets under
``docs/source/Support/bskReleaseNotesSnippets``. At release time:

#. Compile the current snippets:

   .. code-block:: bash

      make -C docs release-notes-snippets

#. Replace the active ``_compiled_latest.rst`` include in
   ``bskReleaseNotes.rst`` with the generated bullet content. Leave the include
   present but commented so it can be re-enabled for the next beta cycle.
#. Add the release date to the active headings in ``bskReleaseNotes.rst`` and
   ``bskKnownIssues.rst``.
#. Review the notes for duplicate, internal-only, or unclear entries and verify
   that every RST reference resolves.
#. Delete only the consumed snippet files. Preserve ``README.md``, the compiler
   script, the sample, and other underscore-prefixed support files.

For a patch release, create dated ``2.X.Y`` sections for the selected patch
notes and known issues without disturbing newer ``develop`` release content.

.. _bsk-release-task-local-validation:

Run Clean Local Validation
~~~~~~~~~~~~~~~~~~~~~~~~~~

Run the validation from the release branch or patch branch being tagged:

.. code-block:: bash

   python conanfile.py --clean --opNav True --mujoco True
   python run_all_test.py
   make -C docs clean
   make -C docs html SPHINXOPTS="-W --keep-going"

The build, test suite, and documentation build must complete without errors or
warnings. See ``docs/source/Support/User/FAQ.rst`` for clean-build details and
``docs/source/Support/Developer/createHtmlDocumentation.rst`` for documentation
prerequisites.

.. _bsk-release-task-rc:

Validate a Release Candidate
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Use a release candidate when the wheel pipeline or release payload should be
tested through TestPyPI before production publication:

#. Set ``bskVersion.txt`` to ``2.X.YrcN`` and commit the change on the branch
   being validated.
#. Run :ref:`clean local validation <bsk-release-task-local-validation>` and
   the appropriate GitHub Actions validation workflow.
#. Tag the tested commit ``v2.X.YrcN`` and push the tag. ``Publish Wheels``
   routes the artifacts to TestPyPI.
#. Install and validate the TestPyPI artifacts. If changes are required,
   increment ``N`` and repeat with a new commit and tag.
#. After the candidate passes, set ``bskVersion.txt`` to ``2.X.Y``, commit the
   final-version change, and repeat local and CI validation before creating the
   final tag.

Do not move or reuse a release-candidate tag after it has been pushed.

.. _bsk-release-task-tag:

Create and Push a Release Tag
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create the tag on the exact validated commit: ``master`` for a major release
or ``patch/v2_X_x`` for a patch release. Push only after confirming the final
version and commit.

.. code-block:: bash

   git status --short
   git tag v2.X.Y
   git push origin v2.X.Y

Tag pushes are publication triggers. Treat pushed tags as immutable; if a
published version is wrong, correct the problem with a new version rather than
moving or reusing the tag.

The ``Publish Wheels`` workflow builds all supported wheels and the source
distribution. Do not continue until every job succeeds and the expected
version is available from PyPI or TestPyPI.

.. _bsk-release-task-patch-docs:

Deploy Patch Documentation
~~~~~~~~~~~~~~~~~~~~~~~~~~

Patch tags do not automatically deploy documentation because the patch branch
is neither ``master`` nor ``develop``. After publishing a patch:

#. On the `Basilisk Actions page
   <https://github.com/AVSLab/basilisk/actions>`_, manually run ``Merge to
   master or develop`` using ``patch/v2_X_x``. This deploys the patch
   documentation at the site root.
#. The root deployment replaces the GitHub Pages contents. Run ``Merge to
   master or develop`` again using ``develop`` to restore the developer
   documentation under ``/develop``.
#. Verify both the root release documentation and the ``/develop``
   documentation after the workflows finish.

.. _bsk-release-task-github-release:

Create the GitHub Release
~~~~~~~~~~~~~~~~~~~~~~~~~

After package publication succeeds:

#. Open the repository's **Releases** page and select **Draft a new release**.
#. Select the published ``v2.X.Y`` tag, regardless of which branch contains
   it.
#. Set the title to ``Basilisk v2.X.Y``.
#. Add the release notes manually or use GitHub's generated notes as a starting
   point. Remove unresolved RST-only markup such as ``:ref:`` roles.
#. Publish the GitHub Release.

.. _bsk-release-task-forward-port:

Forward-Port Patch Fixes
~~~~~~~~~~~~~~~~~~~~~~~~

Never implement the same fix independently on both the patch branch and
``develop``. Develop it once, then cherry-pick that commit to the other branch.
Do not merge ``develop`` wholesale into ``patch/v2_X_x``; maintenance branches
must remain limited to approved fixes for their release line.

Release Safety Rules
--------------------

* Do not create a tag from an uncommitted or unvalidated version change.
* Do not move or reuse a pushed release tag.
* Do not merge ``develop`` wholesale into a maintenance branch.
* Keep patch cherry-picks small, focused, and traceable to their original
  commits.
* Verify package and documentation publication before announcing a release.
