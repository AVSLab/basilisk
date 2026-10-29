# Known Issues Snippets

This folder is the source of truth for contributing known-issue entries.
Known issues cover issues affecting tagged Basilisk releases, including their
fixes and workarounds. New entries do not require editing the version-series
pages under `bskKnownIssues/` or the `bskKnownIssues.rst` index.

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
The blank source lines between bullets do not add spacing to the rendered list.

## Building and reviewing

Sphinx compiles both snippet collections automatically before reading pages.
This includes `make html`, `make.bat html`, direct Sphinx builds, and single-page
previews. The active series page, currently `bskKnownIssues/2.13.rst`, includes
the generated `bskKnownIssuesSnippets/_compiled_latest.rst` under
`Version |release|`. Its relative include is
`.. include:: ../bskKnownIssuesSnippets/_compiled_latest.rst`.
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
replace the active include in `bskKnownIssues/2.X.rst` with the reviewed content
under a literal version and release date, newest first. Group longer lists under
the topic headings and ordering in `../Developer/bskReleaseGuide.rst`, adding
more specific subheadings within large topics when useful. Keep each entry in
one topic with its impact, workaround, and references together; short patch
lists can remain simple bullets. Add a permanent label
such as `bsk-known-issues-2-13-0` above the finalized section, and retain existing
version labels and heading anchors. Remove provisional entries for issues
introduced and fixed entirely within the beta cycle if they never affected a
tagged release.
Delete consumed snippets only after their content is saved in the version
history. Preserve this README and the underscore-prefixed sample. Builds never
archive or delete snippets.

For the next development cycle in the same series, create a new
`Version |release|` section above the finalized section and place the include
there. For a new series, create its page and add it at the top of the
`bskKnownIssues.rst` index. Move the `bsk-known-issues-current` label to the new
active section so the development bookmark follows it. Keep exactly one active
include for known issues, independently of the release-note include.

For a maintenance release, place the dated patch section in the matching series
page; for example, `2.12.1` belongs above `2.12.0` in `2.12.rst`. On the
maintenance branch, move `bsk-known-issues-current` to the newly finalized patch
section, keeping only one copy of that label. Preserve permanent version labels
on older sections so their links still work. The legacy `#version-release`
bookmark will then open the latest patch. When forwarding patch history to
`develop`, leave its current label on the active beta section.

Consume only entries for the fixes on that branch. Do not clear snippets for
newer development work. Keep the `1.x-and-earlier.rst` archive last in the index.

## Updating an existing issue

- **Pending entries:** Update the existing snippet when a later PR changes
  the same issue's description or workaround. Keep one consistent description
  of the current behavior; do not add a second entry that
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
`../Developer/bskReleaseGuide.rst`.

## Migrated 2.13 beta entries

The existing 2.13 beta bullets were moved into individual snippets without
changing their text. Their fixed `2.13-NNN-` filename prefixes preserve the
original order, including references to neighboring entries. These prefixes
are only for this one-time migration: do not renumber them or continue the
sequence for new entries. Use the issue-based or descriptive filenames above
for new snippets. Earlier tagged-release sections are preserved in the
version-series pages under `bskKnownIssues/`.
