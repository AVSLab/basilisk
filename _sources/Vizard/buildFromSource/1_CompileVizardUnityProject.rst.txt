Building the Vizard Unity Project
=================================

These instructions allow you to compile and run Vizard within the Unity application.

#. **Install the Unity Hub.** If it is not yet installed, visit `Unity <https://unity.com>`__
   and follow the platform-specific instructions to install the Unity Hub.

#. **Install the Unity 6000.0 LTS editor.** Vizard is currently based on Unity
   ``6000.0.68f1``.

   a. In Unity Hub, click ``Installs`` on the left side.
   b. Click ``Install Editor`` in the upper-right corner.
   c. Click the ``Archive`` tab at the top of the panel. Click the ``download archive link`` to go to the Unity Archive website.
   d. On the Unity download archive webpage, make sure Unity 6 is selected and then select ``LTS`` from the row of options.
   e. Scroll down to the 6000.0.68f1 (released February 18, 2026) and click Install to the right.
   f. A dialog box will ask to ``Open Unity Hub?`` Click the ``Open Unity Hub`` button.
   g. Select the desired Unity Editor for your platform.
   h. A panel will open with ``Install Unity 6.0 (6000.0.68f1)`` at the top. There will be a list of additional installation options, including platform-specific build support. Install any desired optional components, such as:

      * Visual Studio Code
      * Linux Build Support (Mono)
      * macOS Build Support (IL2CPP)
      * Windows Build Support (Mono)

   i. Click Install to have Unity Hub install the Editor and all additional download options selected.

#. **Open the Vizard Unity project.**

   a. Start Unity Hub.
   b. Click the ``Add`` dropdown in the upper-right corner and select
      ``Add Project From Disk``.
   c. Navigate to ``Vizard/VizardUnityProject`` and click ``Open``.
   d. In the Unity Hub project list, click the newly added project to open it.

   .. note::

      If your installed Unity 6000.0 version does not match the version last used by the
      repository, Unity will ask you to confirm opening the project in a non-matching
      editor. Click ``Yes`` to continue. You may need to resolve issues arising from the
      difference in  Unity Editors.

#. **Load the startup scene.** After ``VizardUnityProject`` finishes importing, type
   ``VizardStartupScene`` into the Project search bar and double-click the scene asset
   to open it.
#. **Install the TMP Essentials Unity package.** When Unity displays the ``TMP Importer``
   panel, click ``Import TMP Essentials``. The ``TMP Examples & Extras`` package is
   optional and can be skipped and the panel closed.

   .. note::

      To force the ``TMP Importer`` panel to appear, click inside the VizardStartupScene
      hierarchy to inspect the StartUpCanvas>Panel>VersionText.

#. **Test the local installation.** Press the Unity ``Play`` button and, in the Game
   window, use ``Select`` to open a Basilisk scenario ``.bin`` file and confirm the
   setup works.
#. **Optional: Install a C# IDE.** A C# IDE is recommended for script editing. Both
   Visual Studio and JetBrains Rider provide optional packages for Unity development.

.. important::

   The atmosphere shader materials for Earth, Mars, and Venus, together with
   installation instructions, are available through the ``Vizard_HD_Materials``
   bundle listed on :ref:`vizardDownload`.
   The atmosphere and ring shader used in Vizard are adapted from the
   `Planet Shader and Shadowing System <https://assetstore.unity.com/packages/vfx/shaders/planet-shader-and-shadowing-system-49693>`__
   package available in the Unity Asset Store.
