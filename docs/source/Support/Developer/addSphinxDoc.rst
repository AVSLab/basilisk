
.. _addSphinxDoc:

Using Sphinx to Document Basilisk Modules and Folders
=====================================================

.. note::

   This document assumes you are already familiar with the Restructured Text format.  Some sources for help can be found at:

   - `restructured Text <http://docutils.sourceforge.net/rst.html>`__
   - `Sphinx Documentation <http://www.sphinx-doc.org/en/master/>`__
   - `Quickguide for restructured text <http://docutils.sourceforge.net/docs/user/rst/quickref.html>`__

Adding Documentation to a Basilisk Module
-----------------------------------------
Assuming you want to document a Basilisk module called `genericModule`.  This means that the folder ``genericModule`` contains the files:

- ``genericModule.c/cpp``
- ``genericModule.h``
- ``genericModule.i``

Simply add the desired module documentation as ``genericModule.rst`` to this folder.  The :ref:`cModuleTemplate` has a sample module documentation file that you can copy into your folder.  This content will be parsed ahead of the module function descriptions.  When running ``cmake`` the ``genericModule.rst`` file should be included in the IDE such as Xcode if the module is a C++ module.  The ``*.rst`` is not shown in the IDE if it is a C-module. Rust module documentation follows the same ``<moduleName>.rst`` convention; see :ref:`rustModules`.


Module Type Labels
------------------
The generated HTML pages for BSK modules include a module type label next to
the page title.  The documentation build infers this label from the module
source files and applies ``C``, ``C++``, ``Python``, or ``Rust`` automatically. Module
documentation files do not need to add this label manually.

The labels are only applied to BSK module pages under ``src/fswAlgorithms`` and
``src/simulation``.  The template modules under ``src/moduleTemplates`` are also
tagged for developer reference.  Documentation pages for ``src/architecture``,
message definitions, ``_GeneralModuleFiles`` support code and other support
files use regular untagged titles.


Automatic Module Catalog
------------------------

The :ref:`Documentation landing page <Folder_Documentation>` includes a
searchable catalog of C, C++, Python, and Rust modules under
``src/fswAlgorithms``, ``src/simulation``, and ``src/moduleTemplates``.
Developers do not maintain a catalog entry or register their modules separately.
The documentation build uses the same generated module titles and language
labels described above; the category comes from the module's folder.

The description is a short excerpt of the first paragraph in the module's
``Executive Summary``. Keep that paragraph focused on what the module does.
For older pages without this heading, the build uses the opening paragraph
when available, or a link-only description. Unit tests, helper files, and
architecture support pages are not listed as modules.

The search field matches the module name, category, language, and full authored
module guide, including the detailed description and user guide. Only the
displayed excerpt is shortened. Generated API listings and auxiliary unit-test
pages are excluded from this search; no manually maintained keywords are needed.

After adding, removing, or editing a module, rebuild the documentation as
described in :ref:`createHtmlDocumentation`. The catalog updates automatically,
including during incremental builds. Do not edit the generated
``docs/source/Documentation/index.rst`` or the catalog's HTML output.


Automatic Example Usage Links
-----------------------------

The module's ``Auxiliary Files`` box can include up to three ``Example usage``
links. These are generated from documented ``scenario*.py`` files under
``examples/``; developers do not maintain a separate list. Top-level scenarios
are selected first, then scenarios in subfolders, alphabetically by filename
within each group.

The build recognizes ordinary Basilisk imports and direct constructor calls,
including import aliases. It does not run scenarios, match comments or keywords,
or follow setup helpers in other files. Dynamic imports, factories, and
ambiguous or reassigned aliases are not inferred. These links demonstrate usage;
they are not a ranking of teaching quality or a complete list of simulations
using the module. Authors can recommend a particular introductory scenario in
the module's user guide.

Rebuilding the documentation refreshes the links automatically when scenarios
are added, removed, or edited.


Documenting Module I/O Messages
--------------------------------
Module documentation can use the ``bsk-module-io`` directive to generate both a
Graphviz module I/O diagram and the standard Basilisk I/O message table from
one RST block.  This keeps the visual diagram and table entries synchronized.
The diagram module element uses the inferred module type label colors.  If the
directive is used outside a generated module page, the ``:module-type:`` option
can be set to ``C``, ``C++``, ``Python``, or ``Rust``.

.. code-block:: rst

   .. bsk-module-io:: GenericModule
      :caption: Module I/O Messages
      :module-type: C++

      input dataInMsg DataMsgPayload
         Input data message.

      output dataOutMsg DataMsgPayload
         Output data message.

Each entry begins with ``input`` or ``output``, followed by the module message
variable name and the payload type.  The indented text below the entry is used
as the table description.  Payload types are automatically rendered as
``:ref:`` links in the generated table.


Adding Documentation to a Basilisk Folder
-----------------------------------------
Some modules in Basilisk are organized into sub-folders.  If you want to add documentation to a particular sub-folder, as is done with :ref:`Folder_power` found at ``src/simulation/power``, then add the name the restructured text file inside that folder and call it ``_doc.rst``.  If a file with this name is found, then its contents will be parsed ahead of show the folders files or sub-folders.


Overriding the Folder's Auto-Generated ``index.rst`` File
---------------------------------------------------------
In some cases, such as with the Basilisk example scripts folder in ``basilisk/examples``, we want to over-ride the auto-generated ``index.rst`` file with a custom file to control how the folder contents is rendered.  This is done by placing a restructured text file called ``_default.rst`` inside this folder.  The sample output can be found in :ref:`examples`.
