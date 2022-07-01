
.. _vizardDownload:

Download
========

.. image:: /_images/static/basiliskVizardLogo.png
       :align: right
       :scale: 50 %

The latest Vizard application binaries binaries can be directly downloaded using the following links:

- :download:`Vizard macOS (Universal) <https://hanspeterschaub.info/bskFiles/Vizard_macOS.zip>`
- :download:`Vizard Linux <https://hanspeterschaub.info/bskFiles/Vizard_Linux.zip>`
- :download:`Vizard Windows <https://hanspeterschaub.info/bskFiles/Vizard_Windows64.zip>`


.. warning::

    The custom Unity asset loading requires Vizard v2.1.1 or later.


The following optional downloads contain models for a range space objects that are too large to
include in the main Vizard binary. You download the desired binaries for the platform you are using
and put them in a specific file location.  On start-up, Vizard checks for their presence and will
load in the space object model if needed.

The bundles depend on the Unity version used to build Vizard.  After opening Vizard, go to ``File/About Vizard``
to see what version of Unity was used to build the application.  Select the appropriate year in the download table.

The bundle must be installed in specific directories on each platform.  On macOS install the bundle in::

    ~/Library/Application Support/Vizard/Vizard/Resources/CustomModels

On Linux, install the bundle contents in::

    home/your_user_name/.config/unity3d/Vizard/Vizard/Resources/CustomModels

On Window, install the bundle contents in::

    C:/Users/your_user_name/AppData/LocalLow/Vizard/Vizard/Resources/CustomModels



.. list-table:: Unity 2020 Asset Download
    :widths: 25 30 15 15 15
    :header-rows: 1

    * - Asset Name
      - Description
      -
      -
      -
    * - asteroid
      - Asteroid model package including high fidelity models of Bennu, Ryugu, Itokawa, as well as some
        generic asteroid models
      - :download:`macOS<https://hanspeterschaub.info/bskFiles/Assets/asteroids_macOS.zip>`
      - :download:`Linux<https://hanspeterschaub.info/bskFiles/Assets/asteroids_Linux.zip>`
      - :download:`Windows<https://hanspeterschaub.info/bskFiles/Assets/asteroids_Windows.zip>`
    * - Martian Moons
      - Package including models for Phobos and Deimos
      - :download:`macOS<https://hanspeterschaub.info/bskFiles/Assets/martianmoons_macOS.zip>`
      - :download:`Linux<https://hanspeterschaub.info/bskFiles/Assets/martianmoons_Linux.zip>`
      - :download:`Windows<https://hanspeterschaub.info/bskFiles/Assets/martianmoons_Windows.zip>`
