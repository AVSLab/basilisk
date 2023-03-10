
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
You need to have a command line version of Doxygen installed on your system. The Doxygen `download
page <https://www.doxygen.nl/download.html>`__ contains a range of pre-compiled binaries for many different platforms.

On macOS the `Homebrew <https://brew.sh>`__ tool is also a very
convenient method to install Doxygen by typing in the terminal::

   brew install doxygen

See :ref:`installOptionalPackages` to learn what python packages must be installed.

Making the HTML Documentation Folder
------------------------------------
To create the HTML documentation with all the associated scenario
figures, be sure to run ``pytest`` first from within the ``/src``
directory.

Next, in a terminal window switch to the ``docs`` folder::

    cd docs

Finally, type the following command to build the HTML documenation::

    make html

The final html documenation folder is stored in ``docs/build/html``.

To clean out the sphinx generated documents and folder use::

    make clean

To open the HTML index file and view the documentation in the browser use::

    make view
