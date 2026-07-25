
.. _createHtmlDocumentation:

Creating the HTML Basilisk Documentation using Sphinx/Doxygen
=============================================================

Documentation Description
-------------------------
The `Sphinx <https://pypi.org/project/Sphinx/>`__ and `Doxygen <http://doxygen.nl>`__ software packages provide an elegant method to both include code explanations, definitions and module documentation, but also to create a full HTML based documentation folder for a software project. An online copy of this HTML documentation is hosted at `AVS Basilisk web site <http://hanspeterschaub.info/bskMain.html>`__ with the `Documentation <http://hanspeterschaub.info/bskHtml/index.html>`__ page.

 .. image:: /_images/static/bskHTML.png
 	:align: center

Tool Requirements
-----------------
You need to have command line versions of Doxygen and Graphviz installed on
your system. The Doxygen `download page <https://www.doxygen.nl/download.html>`__
contains a range of pre-compiled binaries for many different platforms.
Graphviz provides the ``dot`` executable used by Sphinx to render module I/O
diagrams.

On macOS the `Homebrew <https://brew.sh>`__ tool is also a very
convenient method to install these tools by typing in the terminal::

   brew install doxygen graphviz

On Ubuntu or Debian Linux systems these tools can be installed with::

   sudo apt install doxygen graphviz

If you are using a conda environment, Graphviz can also be installed with::

   conda install conda-forge::graphviz

You can verify that Graphviz is available with::

   dot -V

To install the required python packages run the command::

    pip install -r requirements_doc.txt

Making the HTML Documentation Folder
------------------------------------
First generate the test plots::

    cd src

    pytest -n auto

Next, in a terminal window switch to the ``docs`` folder::

    cd docs

Finally, type the following command to build the HTML documentation::

    make html

The final HTML documentation folder is stored in ``docs/build/html``.

To open the HTML index file and view the documentation in the browser use::

    make view

To clean out the sphinx generated documents and folder use::

    make clean

Including Generated Rust Module APIs
------------------------------------

Rust module pages can include the C-compatible configuration interface
generated from their Rust source. Configure and build Basilisk with Rust
module support before running Sphinx:

.. code-block:: console

    python conanfile.py --rustModules True
    cd docs
    make html

Sphinx looks for the generated headers in ``dist3/rust_headers`` and sends
each available module header through the existing Doxygen/Breathe pipeline.

The generated API sections are omitted when a Rust module header is
unavailable, so Rust remains optional for normal documentation builds.

If Basilisk was built in a non-default directory, set
``BSK_RUST_HEADER_DIR`` to that build's ``rust_headers`` directory before
running Sphinx.

Rendering a Single Documentation Page
-------------------------------------
To quickly preview one documentation page while editing it, run ``make`` with
the page path from the ``docs`` folder. For example, to render the release guide
page use::

    make source/Support/Developer/bskKnownIssues.rst

This builds only the requested source page into ``docs/build/html`` and avoids
regenerating the auto-created module documentation source files. This mode is
intended for local editing previews; cross-page links and navigation can be
incomplete because Sphinx does not read the full documentation tree.
