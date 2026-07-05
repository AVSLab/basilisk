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
   - Follow the checkout recommendation in `docs/source/Support/Developer/bskModuleCheckoutList.rst`

5. **MRP attitude typing**
   - For new or modified C++ code, use `Eigen::MRPd` for values that semantically represent Modified Rodrigues Parameter attitudes, such as `sigma_*` attitude variables, state values, properties, and function parameters.
   - Avoid using generic `Eigen::Vector3d` for MRP attitudes unless the code is crossing a boundary that requires raw vector storage, such as message payloads, C arrays, state/property containers, SWIG/Python-facing initialization fields, or generic math APIs.
   - Prefer MRP-specific helpers and methods, such as `cArray2EigenMRPd`, `eigenMRPd2CArray`, `.coeffs()`, `.Bmat()`, `.toRotationMatrix()`, and `.shadow()`, instead of casting MRPs through `Eigen::Vector3d`.
   - Do not convert unrelated three-component physical vectors, such as position, velocity, acceleration, angular rate, force, torque, or magnetic field vectors, to `Eigen::MRPd`.

6. **String and logging security review**
   - For new or modified C/C++ code, check for unsafe writes to fixed-size C buffers, including `strcpy`, `strcat`, `sprintf`, unbounded `%s` scans, and `strncpy` uses that can silently truncate or omit null termination.
   - Prefer bounded formatting/copying APIs such as `snprintf` with `sizeof(destination)` when writing to fixed-size buffers.
   - If a user- or module-supplied string is too long for a fixed-size Basilisk payload field, emit `BSK_ERROR` so execution stops immediately rather than silently truncating.
   - For `bskLog()` and other printf-style logging calls, ensure the format string is a string literal. Dynamic text must be passed through a literal format such as `"%s"`.
   - When a PR fixes one instance of a buffer-write or format-string issue, search nearby BSK modules and related payload paths for the same pattern and flag or fix matching issues.

7. **PR metadata requirements**
   - For normal PRs, include one release-note snippet in:
     `docs/source/Support/bskReleaseNotesSnippets/`.
   - Ensure snippet content follows:
     `docs/source/Support/bskReleaseNotesSnippets/README.md`.
   - Do not edit `docs/source/Support/bskReleaseNotes.rst` directly for normal PRs.
   - If there is a BSK fix, make sure it is mentioned in the `docs/source/Support/bskKnownIssues.rst` file.

8. **SysModel documentation requirement**
   - Every new SysModel must include a corresponding `.rst` documentation file.
   - Pattern new module docs after:
     `src/moduleTemplates/cModuleTemplate/cModuleTemplate.rst` or
     `src/moduleTemplates/cppModuleTemplate/cppModuleTemplate.rst`.
   - For new or modified BSK module `.rst` files, ensure module input/output message documentation uses the `.. bsk-module-io::` directive rather than hand-written module I/O tables or standalone module I/O SVG images.

9. **Basilisk Module Creation**
    - Ensure new Basilisk modules have a unit test
    - Unit test method needs to have documentation strings
    - Ensure the copyright statement is in new files using the current year


10. **Basilisk Example Files**
   - If an example scenario is included in the `examples` folder, ensure this example has a unit test in `src/tests` that imports and runs this example file.
   - If the example file includes the `enableUnityVisualization()` method, ensure the `saveFile` argument is included but commented out.
   - Ensure new Python example scripts are linked in `examples/_default.rst`.
   - Ensure any data importing is done in a robust manner supporting Linux, Windows and macOS
