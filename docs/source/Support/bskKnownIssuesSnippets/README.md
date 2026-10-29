<!-- Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder -->

# Known Issues Snippets

This folder is the source of truth for contributing known-issue entries.
Known issues cover issues affecting tagged Basilisk releases, including their
fixes and workarounds. New entries no longer require editing `bskKnownIssues.rst`.

## When an entry is required

- Add or update an entry when a PR fixes, changes, or documents an issue or
  workaround affecting a tagged release. An issue discovered during a beta
  cycle still needs an entry if it also affects a tagged release.
- An issue introduced and fixed by a commit within the same beta cycle needs
  no known-issue entry if it never affected a tagged release.
- If an issue remains unresolved and will be present in the next tagged
  release, document the limitation and any workaround before tagging.

The beta-cycle exemption applies only to known issues. Follow
`../bskReleaseNotesSnippets/README.md` separately to determine whether a
release-note snippet is required.

## What to add

1. Add a meaningfully named `.rst` file, preferably using the issue number and
   a short description, such as `1658-thruster-control-axes.rst`. Choose a unique
   suffix when separate PRs document distinct aspects of the same issue. When
   revising the same issue's pending description or workaround, update its
   existing entry as described below. Do not use sequence numbers that require
   coordinating with other branches.
2. Describe the affected behavior, the fix or remaining limitation, and any
   workaround or migration steps. Identify the affected tagged release versions
   when known, and include an issue number when available.
3. Start each entry with `- ` and use valid reStructuredText. Wrapped lines,
   additional paragraphs, nested lists, and code blocks must remain indented
   inside that bullet. See `_sample_known_issues_snippet.rst`.
4. A release-note snippet may also be required. Release notes summarize the
   change; known issues explain its impact and workarounds. The compiler does
   not copy entries between the two collections.

Files beginning with `_`, non-RST files, and empty files are ignored. The shared
compiler sorts snippets by filename without regard to case, inserts a blank
line between files, and preserves internal indentation and blank lines.

## Building and reviewing

Sphinx compiles both snippet collections automatically before reading pages.
This includes `make html`, `make.bat html`, direct Sphinx builds, and single-page
previews. The known-issues page includes the generated
`bskKnownIssuesSnippets/_compiled_latest.rst` under `Version |release|`.
The generated file is ignored by Git; commit the authored snippets instead.

Full documentation builds fail if either nonempty collection has no active
include. Sphinx checks the includes it actually parsed; comments and code
examples do not count. Empty collections need no include after finalization.
Single-page previews still compile snippets but skip this full-build check
because the selected page may not use either collection.

To compile without building the documentation, run from the repository root:

```sh
make -C docs snippets
make -C docs known-issues-snippets
```

On Windows, run `make.bat snippets` or `make.bat known-issues-snippets` from
`docs`. The shared compiler also runs without Sphinx or Basilisk installed:

```sh
python docs/source/_ext/rst_snippets.py
python docs/source/_ext/rst_snippets.py known-issues
```

## Release lifecycle

Known issues retain the existing version-history model. Pending snippets can be
written during beta development to describe fixes for issues in tagged releases
or limitations that will ship in the next tagged release. They are not an
automatically maintained registry of open development bugs. An unresolved
limitation affecting a tagged release must remain documented where users can
find it. Consuming a snippet does not mean the issue was resolved.

Before tagging a release, compile and review its known-issue entries, then
replace the active include in `bskKnownIssues.rst` with the reviewed content
under a literal version and release date. Preserve relevant existing inline
entries in that section. Remove provisional entries for issues introduced and
fixed entirely within the beta cycle if they never affected a tagged release.
Delete consumed snippets only after their content is saved in the version
history. Preserve this README and the underscore-prefixed sample. Builds never
archive or delete snippets.

For the next development cycle, create a new `Version |release|` section above
the finalized section and place the include there. Keep exactly one active
include for known issues, independently of the release-note include.

For a maintenance release, consume only entries for the fixes on that branch.
Do not clear snippets for newer development work.

## Updating an existing issue

- **Pending entries:** Update the existing snippet when a later PR changes
  the same issue's description or workaround. During adoption, update the
  existing inline entry in the current `Version |release|` section of
  `bskKnownIssues.rst` if it has not yet moved to a snippet. Keep one consistent
  description of the current behavior; do not add a second entry that
  contradicts or supersedes the first.
  Preserve still-relevant impact, migration instructions, and references.
  If a provisional entry concerns an issue introduced and fixed entirely within
  the beta cycle, remove it once confirmed that no tagged release was affected.
- **Published entries:** Add a new snippet describing the change in the new
  release, referring to the earlier version or issue number. Keep the published
  history intact unless correcting an error in that historical text.

Edits to the same pending entry can still conflict and need reconciliation;
separate snippets avoid conflicts between unrelated changes. Filename order does
not establish which description supersedes another.

The complete release procedure is in
`../Developer/bskReleaseGuide.rst`. Existing inline entries are deliberately
retained during adoption so this workflow does not require rewriting history.
