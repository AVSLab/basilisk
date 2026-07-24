Optional: Install Native File Browser
=====================================

Vizard includes a basic built-in file browser, but installing the following Unity
Asset Store package is recommended to provide a platform-native file browser.

#. If platform-native file browser support is desired, purchase the
   `Crosstales File Browser PRO <https://assetstore.unity.com/packages/tools/utilities/file-browser-pro-98713>`__
   asset from the Unity Asset Store.
#. In the Unity Editor, open the Package Manager through ``Window / Package Manager``.
   Make sure the Editor is not in play mode.
#. In the Package Manager panel, click ``My Assets`` in the left-hand list.
#. Select ``File Browser PRO``.
#. In the right panel, click ``Import 2024.1.1 to project`` or a newer version if desired.
#. In the ``Import Unity Package`` panel, click ``Import``.

   .. note::

      It is recommended to import the entire File Browser PRO package unless you are
      already familiar with its contents.

#. When the ``FB PRO`` panel appears, close it.
#. Open the Unity Player settings through ``Edit / Project Settings`` and select ``Player``.
#. Scroll to ``Scripting Define Symbols``, click ``+``, add ``USE_NATIVE_FILE_BROWSER``,
   and click ``Apply``.
#. Press ``Play`` in the Unity Editor and then select ``Select`` to test the native file browser.
