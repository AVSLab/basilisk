.. toctree::
   :hidden:
.. role:: raw-latex(raw)
   :format: latex
..

.. _cmakeOptions:

Installing With Cmake Configuration Options
===========================================

This documents discusses the various Cmake build options that can be
used when compiling Basilisk. These build options can be set via the GUI
Cmake program or specified via the command line.

.. image:: /_images/static/cmake-no-use.png
   :align: center
   :scale: 40 %

Optional Cmake Flags
--------------------

The Basilisk CMake file contains a series of flags that can be turned on to include additional features within Basilisk or to build in a particular configuration. The flags are listed in the above image. Note that tuning on some flags will invoke the ``conan`` package manager to download and install additional dependencies. Basilisk modules that require particular dependencies will not be compiled unless their support software is installed. Associated BSK module unit tests are skipped if the required software is not provided through these Cmake flags.

The following table summarizes the possible flags:

   ===================== ======== ==============
     cmake Flag          Default  Description
   ===================== ======== ==============
   ``USE_PYTHON3``       ``ON``   enables Basilisk to be compiled for Python 3
   ``USE_PROTOBUFFERS``  ``ON``   installs the `Google protobuffers <https://developers.google.com/protocol-buffers/>`__ package via ``conan``
   ``USE_ZMQ``           ``ON``   installs the `0MQ distributed messaging system <http://zeromq.org>`__ via ``conan``
   ``USE_OPENCV``        ``OFF``  installs the `OpenCV <https://opencv.org>`__ package via ``conan``
   ``USE_COVERAGE``      ``OFF``  turns on code coverage when compiling with GCC on a Linux platform
   ===================== ======== ==============

This table summarizes what ``cmake`` flags are required to get particular Basilisk features:

   ================= ========
     Feature         Flags
   ================= ========
   Python 3          ``USE_PYTHON3``
   Vizard Support    ``USE_PROTOBUFFERS``, ``USE_ZMQ``
   VisNav Modules    ``USE_OPENCV``
   ================= ========

Simplest Basilisk Build
-----------------------

The bare-bones version of Basilisk is invoked by not turning on any
Cmake flags as shown. In the GUI Cmake all the optional compiler flags shown be de-selected as shown in the above figure. To use the command line ``cmake`` command use the regular Basilisk set as discussed in the platform specific Basilisk installation files.

Building for Python 2 or 3
--------------------------

To make ``cmake`` setup a build process to compile Basilisk for Phython 3, then use::

      -DUSE_PYTHON3=ON

If this flag is set to false it is built for Python 2.

Building to Support Vizard Basilisk Playback
--------------------------------------------

To include the ``vizInterface`` module that enables saving off and
communicating with the Vizard Unity based visualization, the two flags ``USE_PROTOBUFFERS`` and ``USE_ZMQ`` must be turned on as shown in the following CMake GUI screen show.

.. image:: /_images/static/cmake-Vizard.png
   :align: center
   :scale: 40 %

To use the command line ``cmake`` command use the
regular Basilisk compile tool with the additional arguments::

   -DUSE_ZMQ=ON -DUSE_PROTOBUFFERS=ON
