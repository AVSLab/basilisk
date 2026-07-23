# AGENTS.md

## Basilisk PR Review Rules

Apply these rules to all **new or materially modified code and documentation** in
this repository.
Do not churn unrelated legacy lines solely to satisfy this file; keep legacy style unless
the surrounding work is already touching that area.

1. **Units on numeric literals**
   - Add a trailing inline unit comment to each numeric literal assignment/definition
     when the quantity has physical meaning.
   - Physical quantities include time, length, mass, angle, rate, force, torque,
     voltage/current, inertia, and similar engineering values.
   - Keep the unit immediately adjacent to the value, typically as an inline comment
     such as `value = 1.2  # [s]`.
   - The exact unit spelling is not important if the unit is clear. Accept common
     forms such as `[N*m]`, `[Nm]`, `Nm`, `[kg*m^2]`, `[kg m^2]`, `[m/s^2]`, and
     `[rad/s]`.
   - Do not flag reasonable unit spellings solely for formatting differences; focus
     on whether units are present and understandable.
   - Use `[-]` for dimensionless physical values when it improves clarity, but do
     not require unit comments for obvious pure math literals or all-zero/identity
     initializers.
   - Unit comments are not required for indexes, counters, array sizes/shapes, loop
     ranges, flags, enum/status values, random seeds, test case identifiers,
     non-physical plotting/layout constants, or other clearly non-physical values.
   - For dense arrays, tables, or function calls, document units once on the variable,
     column, or argument when per-element comments would hurt readability.

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
   - Follow the checkout recommendation in `docs/source/Support/Developer/bskModuleCheckoutList.rst`

5. **String and logging security review**
   - For new or modified C/C++ code, check for unsafe writes to fixed-size C buffers, including `strcpy`, `strcat`, `sprintf`, unbounded `%s` scans, and `strncpy` uses that can silently truncate or omit null termination.
   - Prefer bounded formatting/copying APIs such as `snprintf` with `sizeof(destination)` when writing to fixed-size buffers.
   - If a user- or module-supplied string is too long for a fixed-size Basilisk payload field, emit `BSK_ERROR` so execution stops immediately rather than silently truncating.
   - For `bskLog()` and other printf-style logging calls, ensure the format string is a string literal. Dynamic text must be passed through a literal format such as `"%s"`.
   - When a PR fixes one instance of a buffer-write or format-string issue, search nearby BSK modules and related payload paths for the same pattern and flag or fix matching issues.

6. **PR metadata requirements**
   - Follow `docs/source/Support/bskReleaseNotesSnippets/README.md` for when a
     release-note snippet is required and how snippet files should be formatted.
   - Update `docs/source/Support/bskKnownIssues.rst` when a PR fixes, changes, or
     documents a known user-visible issue or workaround.

7. **SysModel documentation requirement**
   - Every new SysModel must include a corresponding `.rst` documentation file.
   - Pattern new module docs after:
     `src/moduleTemplates/cModuleTemplate/cModuleTemplate.rst` or
     `src/moduleTemplates/cppModuleTemplate/cppModuleTemplate.rst`.
   - For new or modified BSK module `.rst` files, ensure module input/output message documentation uses the `.. bsk-module-io::` directive rather than hand-written module I/O tables or standalone module I/O SVG images.

8. **Basilisk Module Creation**
   - Ensure new Basilisk modules have a unit test
   - Unit test method needs to have documentation strings
   - Ensure the copyright statement is in new files using the current year


9. **Basilisk Example Files**
   - If an example scenario is included in the `examples` folder, ensure this example has a unit test in `src/tests` that imports and runs this example file.
   - If the example file includes the `enableUnityVisualization()` method, ensure the `saveFile` argument is included but commented out.
   - Ensure new Python example scripts are linked in `examples/_default.rst`.
   - Ensure any data importing is done in a robust manner supporting Linux, Windows and macOS
