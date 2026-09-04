
.. _vizardCustomUnityModels:

Custom Unity Models
===================

.. image:: /_images/static/basiliskVizardLogo.png
       :align: right
       :scale: 50 %

By taking advantage of Unity’s Addressable Assets system, users can import their models into the Vizard Public Content Unity project and create custom Addressables bundles. These Addressables bundles are placed in the Vizard application’s Application Support library and the models they contain can be imported at runtime by the Vizard application by populating the ``modelDictionaryKey`` with each model’s unique key.

Using the Unity Editor to import the custom models allows users to leverage Unity’s internal model pre-processing and Vizard’s performance is improved when complex models are loaded from Addressables bundles instead of imported with the runtime OBJ importer.

.. warning::

    The custom Unity asset loading requires Vizard v2.1.1 or later.

To get started, first determine what version of Unity the Vizard application is built with.
Start up Vizard, load a scenario and go to ``File/About Vizard`` to see the Unity version
used to build this program

.. image:: /_images/static/vizardInfo.jpg
       :align: center
       :scale: 50 %


Next, download the corresponding version of the Unity Editor from the  `Unity web site <https://unity.com/releases/editor/archive>`__.
Be sure to click on the tab marked for the version you are looking for
and make sure to include support for all platforms you intend to use the
custom Addressables Bundles. Vizard supports MacOS, Linux, and Windows.

.. warning::

    Addressable bundles MUST be built with the same Unity editor version that was used to build the Vizard application.  The ``About Vizard`` panel shows what version this is.

A public content Unity project has been created to guide users through the import of
custom models and bundling them into Addressables and includes required settings
for successful export of the bundles. This Unity project can be downloaded here :

- :download:`VizardPublicContent <https://hanspeterschaub.info/bskFiles/VizardPublicContentUnityProject.zip>`

Extract and open the ``VizardPublicContent`` project with Unity. If Unity displays a
warning message indicating that ``VizardPublicContent`` was created with an earlier
release of Unity than is installed, select “Choose another Editor version”
and select the local version of that is installed and open the project
with that. A warning will appear that the project is being opened in a
non-matching Editor installation. Click continue.

A guide to importing and preparing models for Addressable bundles can be downloaded here:

- :download:`ImportingCustomModels <https://hanspeterschaub.info/bskFiles/ImportAndPrepareCustomModelForVizard.pdf>`

Instructions for building the custom models Addressables bundles can be downloaded here:

- :download:`ExportingCustomModelBundles <https://hanspeterschaub.info/bskFiles/CreateAndExportCustomVizardContentBundles.pdf>`

The bundles must be rebuilt for each desired target platform. The bundle must then be installed in specific directories on each platform as discussed in :ref:`vizardDownload`.

Use the ``modelDictionaryKey`` keyword to apply custom models to spacecraft objects as
discussed in :ref:`specifySpacecraftCAD_label`.

Also use of the ``modelDictionaryKey`` to apply custom models to celestial body objects as discussed in :ref:`specifyCelestialCAD_label`.

.. important::

    The units of models when applied to spacecraft are assumed to be meters. Models applied to celestial bodes are assumed to be in kilometers.
