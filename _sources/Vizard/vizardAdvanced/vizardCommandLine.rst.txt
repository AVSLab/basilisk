.. _vizardCommandLine:

Command Line Launching
======================

.. image:: /_images/static/basiliskVizardLogo.png
       :align: right
       :scale: 50 %

This page contains a series of terminal commands to open Vizard from the command line on macOS
and provide optional arguments.  Adjust the command line ``open`` command and file name according
to your platform and how you named the Vizard application.  The examples below it is assumed the
application is called ``Vizard.app`` and it is installed in the syste ``/Applications`` folder.

To launch an instance of the `Vizard <vizard>`_ application from the command line::

	open /Applications/Vizard.app

The `Vizard <vizard>`_ application supports the following command line arguments:

#. To launch `Vizard <vizard>`_ application and automatically load a playback binary Vizard data file,
   use the ``-loadFile`` argument followed by the filepath::

	open /Applications/Vizard.app --args -loadFile ~/filepath/filename.bin

#. To launch `Vizard <vizard>`_ application in direct 2-way communication mode without displaying anything to the screen,
   use the argument ``-batchmode`` with the ``-noDisplay`` argument::

	open /Applications/Vizard.app --args -batchmode -noDisplay tcp://xxx.xxx.x.xx.xxxxx

#. To launch `Vizard <vizard>`_ application with a livestreaming connection to DirectComm,
   use the ``-directComm`` argument followed by the TCP address `Vizard <vizard>`_ should connect to::

	open /Applications/Vizard.app --args -directComm  tcp://xxx.xxx.x.xx:xxxxx

   Note that here Vizard will show the scene being rendered which is helpful for debugging and
   demonstrations, but it will slow down the simulation.

   To run Vizard cameras in a headless mode, use the ``-noDisplay`` argument instead.

#. To save a copy of all vizMessages passed to `Vizard <vizard>`_ during the run,
   use the ``-saveMsgFile`` argument. Optionally, this argument can be followed by the desired
   file name that `Vizard <vizard>`_ will name the saved message file on exit::

	open /Applications/Vizard.app --args -saveMsgFile filenameToUse

#. To launch `Vizard <vizard>`_ application and safe the rendering and image transmission times (in ``noDisplay`` mode)
   to the file ``~/VizardData/opNavMetrics.txt``, use::

     open /Applications/Vizard.app --args -saveMetrics -noDisplay tcp://xxx.xxx.x.xx.xxxxx
