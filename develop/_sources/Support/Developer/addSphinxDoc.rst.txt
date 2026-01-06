
.. _addSphinxDoc:

Using Sphinx to Document Basilisk Modules and Folders
=====================================================

.. note::

   This document assumes you are already familiar with the Restructured Text format.  Some sources for help can be found at:

   - `restructured Text <http://docutils.sourceforge.net/rst.html>`__
   - `Spinx Documenation <http://www.sphinx-doc.org/en/master/>`__
   - `Quickguide for restructured text <http://docutils.sourceforge.net/docs/user/rst/quickref.html>`__

Adding Documentation to a Basilisk Module
-----------------------------------------
Assuming you want to document a Basilisk module called `genericModule`.  This means that the folder ``genericModule`` contains the files:

- ``genericModule.c/cpp``
- ``genericModule.h``
- ``genericModule.i``

Simply add the the desired module documentation as ``genericModule.rst`` to this folder.  The :ref:`cModuleTemplate` has a sample module documentation file that you can copy into your folder.  This content will be parsed ahead of the module function descriptions.  When running ``cmake`` the ``genericModule.rst`` file should be included in the IDE such as Xcode if the module is a C++ module.  The ``*.rst`` is not shown in the IDE if it is a C-module.


Adding Documentation to a Basilisk Folder
-----------------------------------------
Some modules in Basilisk are organized into sub-folders.  If you want to add documentation to a particular sub-folder, as is done with :ref:`Folder_power` found at ``src/simulation/power``, then add the name the restructured text file inside that folder and call it ``_doc.rst``.  If a file with this name is found, then its contents will be parsed ahead of show the folders files or sub-folders.


Overriding the Folder's Auto-Generated ``index.rst`` File
---------------------------------------------------------
In some cases, such as with the Basilisk example scripts folder in ``basilisk/examples``, we want to over-ride the auto-generated ``index.rst`` file with a custom file to control how the folder contents is rendered.  This is done by placing a restructured text file called ``_default.rst`` inside this folder.  The sample output can be found in :ref:`examples`.
