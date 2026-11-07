Building a Stand-Alone Vizard Application
=========================================

The following instructions allow you to build a stand-alone version of the Vizard application.

#. In the Unity Editor, open the Build Profiles panel through ``File / Build Profiles``.
#. Select the desired target platform from the list on the left.

   If the desired platform is not listed, return to Unity Hub and install support for
   that platform first.

#. Confirm that both ``VizardStartupScene`` and ``VizardMainScene`` are enabled in the
   scene list. If not:

   a. Select ``Scene List`` from the left-hand panel.
   b. In the Scene tab, enable both scenes by checking their boxes.

#. Click ``Build`` or ``Build And Run`` and select the output location for the new application.

.. note::

   The first build for each target platform takes longer because Unity must compile
   shaders for that platform. Later builds are typically much faster. Rebasing the
   project onto a newer Unity Editor release can also trigger the longer compile time.

.. important::

   Users of the Vizard application must install any desired optional remote Vizard
   Addressables bundles locally on their machine. These bundles are not included in
   the built application.
