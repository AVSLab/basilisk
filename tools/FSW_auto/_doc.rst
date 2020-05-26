.. toctree::
   :hidden:

.. _fswAuto:

Auto-wrapping Flight Software
=============================

``autosetter`` Directory
------------------------

General Description
~~~~~~~~~~~~~~~~~~~
The purpose of this script is to automatically translate the setup code from a Basilisk
Python scenarios into C.  This facilitates the process to move a BSK simulation of algorithms into
a pure C software architecture.

The setup code encompasses:

    1) C modules' variables initialization and
    2) Grouping of modules in tasks

Instructions
~~~~~~~~~~~~
Run the ``auto_setter.py`` file that lives inside the ``autosetter`` directory.  The expected output has
one header and source file containing the setup code written in C will be created in a fresh local
directory named ''sets''


``autowrapper`` Directory
-------------------------
The purpose of this script is to create C++ classes with setters and getters around each C module.
Run the ``auto_wrapper.py`` file that lives inside the ``autowrapper`` directory to execute an example.
The expected output are C++ wrapper classes of the C FSW modules in a fresh local directory named ``wraps``.

