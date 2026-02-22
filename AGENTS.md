# AGENTS.md

## Codex PR Review Rules

Apply these rules to all **new or modified code** in this repository.

1. **Units on numeric literals**
   - Add a trailing inline unit comment to each numeric literal assignment/definition when the quantity has physical meaning.
   - Format: `value = 1.2  # [s]`.
   - Use SI-style bracketed units (examples: `[s]`, `[m]`, `[kg]`, `[rad]`, `[m/s]`, `[N*m]`).

2. **Doc comment format**
   - All new/updated Python docstrings and Doxygen comments must be valid, well-formed reStructuredText (`.rst`) markup.
   - Keep parameter, return, and note sections consistent and renderable by Sphinx/Doxygen.

3. **Spelling quality**
   - No misspellings in newly added/modified identifiers, comments, docstrings, Doxygen blocks, user-facing strings, or docs text.

4. **Coding-guideline conformance**
   - New code must follow Basilisk coding guidelines in:
     `docs/source/Support/Developer/CodingGuidlines.rst`.
   - Respect language-specific foundations referenced there (C/C++: Stroustrup/Sutter style basis; Python: PEP 8), plus Basilisk naming and message conventions.
   - For legacy non-conforming files: keep style consistent for minor edits; use major refactors to move code toward guideline compliance.

5. **PR metadata requirements**
   - For normal PRs, include one release-note snippet in:
     `docs/source/Support/bskReleaseNotesSnippets/`.
   - Ensure snippet content follows:
     `docs/source/Support/bskReleaseNotesSnippets/README.md`.
   - Do not edit `docs/source/Support/bskReleaseNotes.rst` directly for normal PRs.

6. **SysModel documentation requirement**
   - Every new SysModel must include a corresponding `.rst` documentation file.
   - Pattern new module docs after:
     `src/moduleTemplates/cModuleTemplate/cModuleTemplate.rst` or
     `src/moduleTemplates/cppModuleTemplate/cppModuleTemplate.rst`.
