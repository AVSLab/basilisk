Configure Vizard for Virtual Reality (Windows Only)
===================================================

.. note::

   Vizard is configured to run with Unity's OpenXR packages and has been deployed
   with Meta Quest 2 and Quest 3 headsets.

.. note::

   To test with a Meta Quest headset, the
   `Meta Horizon Link <https://www.meta.com/help/quest/1517439565442928/>`__
   application must be installed on the Windows machine, and the Quest headset must
   have `developer mode <https://developers.meta.com/horizon/documentation/unity/unity-env-device-setup/>`__
   enabled.

Configure Vizard for VR
-----------------------

#. Open ``VizardUnityProject`` in the Unity Editor.
#. Install XR packages

   a. Open the Unity Package Manager through ``Window / Package Manager``.
   b. Select ``Unity Registry`` from the left-hand list.
   c. Select ``XR Interaction Toolkit`` from the available packages.
   d. In the package details panel, click ``Install``.
   e. Next, select ``XR Plug-In Management`` from the Unity Registry list.
   f. In the package details panel, click ``Install``.
   g. Close the Unity Package Manager.

#. Enable OpenXR in XR Plug-in Management

   a. Open ``Edit / Project Settings``.
   b. Select ``XR Plug-In Management`` in the left-hand list.
   c. Enable ``OpenXR`` in the list of plug-in providers.
   d. Confirm that ``Initialize XR on Startup`` is enabled.

#. Add the ``VIZARD_OPENXR`` compile argument

   a. In Project Settings, select ``Player``.
   b. Scroll to ``Scripting Define Symbols``, click ``+``, and add ``VIZARD_OPENXR``.
   c. Click ``Apply``.

#. Add VizardVR_MainScene to the scene list

   a. Open the Build Profiles panel through ``File / Build Profiles``.
   b. Select ``Scene List`` from the left-hand list.
   c. Enable ``Scenes/VizardVR_MainScene``. Optionally disable the 2D main scene by
      clearing ``Scenes/VizardMainScene``.

#. Connect the Quest headset

   a. Open the Meta Horizon Link application.
   b. Connect the Quest headset through a cable, or wirelessly if preferred, and enable
      the Link. The headset should show a white waiting room.
   c. Press ``Play`` in the Unity Editor on the desktop. File selection is not currently
      available from within the headset, so playback-file selection or live Basilisk
      connection setup must be done from the desktop Editor window.

Vizard VR Controller Guide
--------------------------

**Controller Guide**

Vizard VR supports Meta Quest 2 and Quest 3 controllers. Changes to the controller
configuration can be made in the ``VizardOpenXR`` input action asset.

.. image:: /_images/static/VizardVR_ControllerGuide.jpg
   :align: center
   :scale: 48 %

**View Menu**

.. image:: /_images/static/VizardVR_ViewMenu.jpg
   :align: center
   :scale: 48 %

**Program Controls Menu**

.. image:: /_images/static/VizardVR_ProgramControlsMenu.jpg
   :align: center
   :scale: 48 %
