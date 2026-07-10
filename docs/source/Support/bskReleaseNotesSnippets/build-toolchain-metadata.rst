- Added ``Basilisk.getBuildInfo()`` and ``Basilisk.printBuildInfo()`` to report build diagnostics and a compiled,
  versioned C/C++ ABI descriptor for an installed Basilisk package.  The public descriptor contract is shared with
  BSK-SDK headers, module-only builds refresh the compiled ABI metadata, and CMake uses the Python interpreter from
  the environment that invoked the build.
