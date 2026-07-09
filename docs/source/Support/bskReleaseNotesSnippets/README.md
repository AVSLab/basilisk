Release Notes Snippets
======================

Purpose
-------
This file is the source of truth for release-note snippet requirements.

Pull requests with user- or developer-facing changes should contribute release notes by
adding a snippet file in this folder instead of editing
``docs/source/Support/bskReleaseNotes.rst`` directly.

User- or developer-facing changes include public APIs, modules, examples,
documentation, build/install behavior, packaging, validation semantics, performance,
and bug fixes users may care about.

A snippet is not required for internal-only refactors, test-only maintenance,
formatting/lint-only changes, comment-only changes, generated-file cleanup, or small
internal plumbing changes with no user/developer-facing behavior, API, documentation,
build/install, packaging, performance, or validation impact.

What to add
-----------
1. Add a new .rst file in this folder when a release-note snippet is required.
2. The filename can be anything meaningful (for example: 1255-tle-osculating.rst).
3. Put one release-note item per line.
4. Each line should be written exactly as it should appear in the release notes.
5. Use valid RST markup when needed (for example, :ref:`moduleName` or inline links).

How snippets are inserted
-------------------------
The file ``docs/source/Support/bskReleaseNotes.rst`` includes:

``docs/source/Support/bskReleaseNotesSnippets/_compiled_latest.rst``

When you run the regular ``make html`` command, this will create the compiled release notes file by running the python file:

``docs/source/Support/bskReleaseNotesSnippets/_compile_release_notes_snippets.py``

The generator reads snippet files in this folder, concatenates them in filename-sorted order,
and writes the compiled output file.

Formatting guidance
-------------------
- Keep each line concise and user-facing.
- Start lines with "- " so they are valid bullet items when merged.
- If a PR adds multiple features/fixes, add multiple lines in the same snippet file.
- If a PR adds three modules, add three separate lines.
- Files named ``README.md`` are ignored.
- Files starting with ``_`` are ignored (reserved for templates/examples/generated files).
- The generated file ``_compiled_latest.rst`` is overwritten by the generator.

Committing to the branch
------------------------
Be sure to commit the release notes snippet(s) to your branch to ensure they are included in the combined release notes of the next released version.


Example
-------
See:

_sample_release_notes_snippet.rst
