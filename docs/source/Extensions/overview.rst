.. toctree::
   :maxdepth: 1
   :hidden:

.. _bskExtensions:

About Extensions
================

.. sidebar:: What is an Extension?

    A Basilisk extension is an independently built Python package containing
    one or more custom C, C++, or Rust modules, custom message definitions,
    and supporting code. It is installed alongside a compatible Basilisk
    release without becoming part of the core Basilisk build. Rust support is
    experimental; see :ref:`writingRustPlugins`.

Basilisk ships a curated set of simulation modules for common astrodynamics
tasks. Extensions let developers add new dynamics models, environment models,
sensors, effectors, algorithms, messages, and other simulation capabilities
without modifying the Basilisk source tree.

An extension can own:

* one or more C, C++, or Rust modules that use Basilisk module APIs;
* custom message payloads and their generated Python bindings;
* shared support code used by those modules;
* pure-Python helpers and modules; and
* its own tests, examples, version, and release process.

The extension normally lives in a separate repository controlled by its
developer or organization. Core Basilisk and the extension can therefore
evolve as separate projects. Extension developers do not need to place files
inside Basilisk, modify core modules, or carry local source changes that can
conflict when updating the Basilisk checkout. An extension must still be
rebuilt when moving to a different Basilisk version because it uses Basilisk's
compiled C/C++ and SWIG interfaces.

Why the Package Boundary Matters
--------------------------------

Independent Ownership
~~~~~~~~~~~~~~~~~~~~~

The extension repository contains the custom source, messages, tests, and
packaging configuration. Its maintainers decide when to change or release it.
Basilisk remains a separately installed dependency rather than a source tree
that must contain every local customization.

Isolated Builds
~~~~~~~~~~~~~~~

An extension has its own build graph. Rebuilding it compiles the affected
extension targets and a small set of support sources supplied by
``bsk-sdk``. It does not rebuild the installed core Basilisk package.

This distinction is especially important for custom messages. Changing an
extension-owned payload regenerates that payload's bindings and recompiles the
extension targets that depend on it. The change does not regenerate or
recompile the core Basilisk message system.

Native In-Process Execution
~~~~~~~~~~~~~~~~~~~~~~~~~~~

The package boundary is a build and distribution boundary, not a process
boundary. Extension modules and Basilisk run in the same Python process and
exchange compatible Basilisk objects and messages directly. There is no IPC
or serialization step on each module update, so a compiled extension retains
the native execution characteristics of a built-in C or C++ module.

Distribution
~~~~~~~~~~~~

An extension can be built as a standard Python wheel and distributed through
PyPI, a private package index, or another artifact store. End users can install
a prebuilt wheel without a compiler or Basilisk source checkout, provided a
wheel exists for their operating system, processor architecture, Python
version, and Basilisk version.

.. _extensions-vs-external-modules:

Extensions vs. Integrated External Modules
------------------------------------------

Basilisk also supports the :ref:`buildExtModules` external-folder option. That
option keeps custom source outside the repository but folds it into a
from-source Basilisk build. The result is one customized Basilisk installation
containing both core and external modules.

.. list-table:: Extension and external-folder build comparison
   :header-rows: 1
   :widths: 27 36 37

   * - Characteristic
     - Extension
     - External-folder build
   * - Project ownership
     - Independent repository, package, and release lifecycle
     - Separate source folder incorporated into one Basilisk build
   * - Basilisk source checkout
     - Not required for a released Basilisk version
     - Required
   * - Build product
     - Separate wheel installed alongside Basilisk
     - One customized Basilisk installation
   * - Normal module rebuild
     - Rebuilds affected extension targets and SDK support sources
     - Rebuilds affected targets inside the integrated Basilisk build
   * - Custom message rebuild
     - Regenerates the extension message bindings and affected extension
       targets; core Basilisk remains untouched
     - Updates the combined message system and can trigger broad regeneration
       and recompilation across the Basilisk build
   * - Runtime execution
     - Native and in-process
     - Native and in-process
   * - Distribution
     - Can publish platform-specific wheels for ``pip install``
     - Distribute the customized build or its source and build instructions
   * - Basilisk API access
     - Uses the headers, base classes, messages, and utilities exported by
       ``bsk-sdk``
     - Builds with the complete Basilisk source tree and can use internal
       implementation details
   * - Version relationship
     - Rebuild against the SDK matching each targeted Basilisk version
     - Core and custom code are rebuilt together
   * - Python import
     - Extension-defined package, such as ``my_extension``
     - ``Basilisk.ExternalModules``

Extensions are the recommended choice for custom modules and messages that can
be built with the interfaces exported by ``bsk-sdk``. The external-folder
option remains useful when custom code must participate in the complete
Basilisk source build, depends on non-exported internals, or must be delivered
as one customized Basilisk implementation.

How Extensions Work
-------------------

Extensions are built with
`bsk-sdk <https://pypi.org/project/bsk-sdk/>`_, a companion package that
contains a versioned subset of Basilisk needed for out-of-tree development:

* public headers and SWIG interface files;
* custom-message generation tools;
* CMake helpers for C and C++ modules; and
* a small set of Basilisk runtime and utility sources compiled into extension
  targets.

The build-time and runtime relationships are different:

.. graphviz::

   digraph extension_arch {
      graph [rankdir=TB, splines=ortho, bgcolor=transparent, nodesep=0.45,
             ranksep=0.55]
      node [shape=box, style="rounded,filled", fontname="Helvetica",
            fontsize=12, margin="0.35,0.18"]
      edge [fontname="Helvetica", fontsize=10]

      subgraph cluster_build {
         label="Build time"
         color="#9aa0a6"
         style="rounded"
         source [label="Extension source\n(modules and messages)",
                 fillcolor="#dce8fb"]
         sdk [label="bsk-sdk\n(headers, generators, CMake, support sources)",
              fillcolor="#d4edda"]
         wheel [label="Extension wheel", fillcolor="#f1f3f4"]
         source -> wheel [label=" compile "]
         sdk -> wheel [label=" build against "]
      }

      subgraph cluster_runtime {
         label="Runtime"
         color="#9aa0a6"
         style="rounded"
         bsk [label="Matching Basilisk wheel", fillcolor="#fff3cd"]
         installed [label="Installed extension wheel", fillcolor="#dce8fb"]
         sim [label="One Basilisk simulation process", fillcolor="#f1f3f4"]
         bsk -> sim [label=" import "]
         installed -> sim [label=" import "]
      }

      wheel -> installed [label=" pip install "]
   }

At build time, ``bsk-sdk`` checks the installed Basilisk version and SWIG
runtime compatibility. It then generates any custom message bindings and
builds the extension's native Python modules. The already installed Basilisk
wheel is not rebuilt.

At runtime, the matching Basilisk and extension wheels are imported into the
same simulation process. This allows extension modules to inherit supported
Basilisk base classes, use built-in and custom messages, and participate in the
normal task scheduling and message-passing system.

Custom Messages
---------------

An extension can define payload headers in its own repository and use
``bsk_generate_messages`` to create the same style of Python message,
recorder, and optional C interface support used by core Basilisk messages. The
generated bindings are packaged inside the extension's Python namespace.

Both extension modules and Python simulation code can import these custom
message types. Because the payload definitions belong to the extension, a
payload change remains local to that extension's build and does not modify the
installed ``Basilisk.architecture.messaging`` package.

Version and ABI Compatibility
-----------------------------

An extension has its own package version, but each compiled wheel targets a
specific Basilisk version. ``bsk-sdk`` versions track Basilisk versions; for
example, ``bsk-sdk==2.11.0`` contains the interfaces from Basilisk ``v2.11.0``.

Extension authors should pin the same Basilisk version in their build and
runtime dependencies:

.. code-block:: toml

   [build-system]
   requires = ["bsk-sdk==2.X.Y", "bsk==2.X.Y"]

   [project]
   dependencies = ["bsk==2.X.Y"]

CMake reports a version or SWIG ABI mismatch while building an extension. For
a prebuilt wheel, the extension's runtime dependency metadata is what prevents
``pip`` from selecting an incompatible Basilisk release. Upgrading Basilisk
therefore requires rebuilding the extension or installing an extension wheel
built for the new version.

Next Steps
----------

* To install an already built extension, see :ref:`extensionsInstall`.
* To build the working example and create an extension of your own, follow
  :ref:`writingExtensions`.
* To build custom modules as part of one integrated Basilisk implementation,
  see :ref:`buildExtModules`.
* For a complete extension containing C++, C, and custom-message examples, see
  the `custom atmosphere extension
  <https://github.com/AVSLab/bsk_sdk/tree/master/examples/custom-atm-extension>`_.
