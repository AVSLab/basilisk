.. _makingModules-3:

Module RST Documentation
========================

Creating the Documentation Page
-------------------------------
Each module should have a plain text RST file in the module folder, along with the ``*.h``, ``*.cpp`` and ``*.i`` files.  This documentation file should provide a description of what the module does, have a table of the input and output messages used, and provide a user guide.  See :ref:`cModuleTemplate` for a sample RST module description file that contains a lot of information on how to document a module.

Testing the Documentation
-------------------------
To test the RST documentation, see :ref:`createHtmlDocumentation` on how to run ``sphinx`` to create the HTML documentation in ``basilisk/docs/build/html``.  Note that building the full documentation can take a while.  At the bottom of the ``basilisk.docs/source/conf.py`` file there are lines that determine what modules are included.  Instead of including the entire source folder, comment out that line and uncomment another line that points just to the new module folder.  This significantly speeds up the documentation build process.  Naturally, don't commit this change to ``conf.py``, but only use it testing the formatting.  Note that if only the single module documentation is built links to other modules etc. won't work.  Be sure to build the entire BSK documentation to give this a final test and read through.


