
.. _createHtmlDocumentation:

Creating the HTML Basilisk Documentation using Sphinx/Doxygen
=============================================================

Documentation
-------------

The `Sphinx <https://pypi.org/project/Sphinx/>`__ and `Doxygen <http://doxygen.nl>`__ software packages provide an elegant method to both include code explanations, definitions and module documentation, but also to create a full HTML based documentation folder for a software project. An online copy of this HTML documentation is hosted at `AVS Basilisk web site <http://hanspeterschaub.info/bskMain.html>`__ with the `Documentation <http://hanspeterschaub.info/bskHtml/index.html>`__ page.
 
 .. image:: ../_images/static/bskHTML.png
 	:align: center
 	


Requirements to Create Local Copy of HTML Documentation
---------------------------------------------------------------

You either need to have a command line version of Doxygen installed or your system. The Doxygen `download
page <http://www.stack.nl/~dimitri/doxygen/download.html>`__ contains a range of pre-compiled binaries for many different platforms.

On macOS the `Homebrew <https://brew.sh>`__ tool is also a very
convenient method to install Doxygen by typing in the terminal::

   brew install doxygen



The BSK Doxygen setup file has the flab ``HAVE_DOT`` set to
true. This allows for the HTML class description images to be vector SVG images. Further, these images are interactive and embed hyperlinks to other related BSK classes. As a result, make sure that the `GraphViz <http://www.graphviz.org>`__ tool is installed and availabe in the default path. On macOS using Homebrew you can simply type::

   brew install graphviz

to install this tool.


The following python packages must be installed via ``pip``::

    pip3 install --user sphinx sphinx_rtd_theme breathe recommonmark


Creating HTML Documentation
---------------------------

To create the HTML documentation with all the associated scenario
figures, be sure to run ``pytest`` first from within the ``/src``
directory.

Next, in a terminal window switch to the `docs` folder::

    cd docs

Finally, type the following command to build the HTML documenation::

    make html

The final html documenation folder is stored in ``docs/build/html``.

