.. toctree::
   :hidden:


.. _installMacOSM1:

Setup On Apple Computer with M1 Processor
=========================================

These instructions allow you to run Basilisk on an Apple computer with the new ``arm64`` architecture, such as with the M1 chip.  All required BSK packages are now available on the ``arm64`` platform with the regular ``pip3``
install command.
We still follow the same general setup as discussed in :ref:`installMacOS`.  However, some extra steps are required and outlined below.

Install Python
--------------
The `Python.org <https://python.org>`__ web site contains a Universal binary for Python 3.9.  You can download the installer package there.

Using Homebrew
--------------
Install `HomeBrew <http://brew.sh>`__ using a Terminal window as normal.  The latest version 3.0 and up natively supports the M1 processor.  You can use ``brew`` to install ``swig``, ``cmake`` to support building Basilisk, and install ``doxygen`` to support building the Basilisk documentation.  Note that ``cmake`` must be at least version 3.20
to propoerly setup the Xcode project for ``arm64``.

