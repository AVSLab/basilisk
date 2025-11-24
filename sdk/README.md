# Basilisk SDK

This package publishes the header-only Basilisk plugin SDK so that external
projects can build Basilisk-compatible plugins without vendoring the full
simulation codebase. It currently ships the ``bsk/sdk.hpp`` helper and a
curated subset of Basilisk's ``architecture`` headers. The package depends on
``pybind11`` and offers convenience accessors,
:func:`bsk_sdk.include_dir` and :func:`bsk_sdk.include_dirs`, for build systems.

To refresh the vendored Basilisk headers from the source tree run::

    python sdk/tools/sync_headers.py
