.. toctree::
   :maxdepth: 1
   :hidden:

.. _pluginsInstall:

Installing Plugins
==================

A Basilisk plugin is a standard Python wheel.  Installing one requires
only ``pip`` — no compiler, no source checkout, no rebuilding Basilisk.

Installing a Plugin
-------------------

.. code-block:: bash

    pip install <plugin-name>

For example, if a plugin is published to PyPI as ``my-atm-plugin``:

.. code-block:: bash

    pip install my-atm-plugin

This installs the plugin and pulls in Basilisk as a dependency
automatically.  Once installed, use it like any built-in Basilisk module:

.. code-block:: python

    from my_atm_plugin import customAtmosphere
    from Basilisk.utilities import SimulationBaseClass

    sim = SimulationBaseClass.SimBaseClass()
    atm = customAtmosphere.CustomAtmosphere()
    sim.AddModelToTask("task", atm)

Version Compatibility
---------------------

A plugin wheel is compiled against a specific Basilisk version.  Always
install matching versions to avoid ABI incompatibilities:

.. code-block:: bash

    pip install "bsk==2.9.1" "my-plugin==2.9.1"

If the versions do not match, the plugin's CMake build will error with a
clear message telling you which versions to align.

Installing the SDK (for plugin authors)
----------------------------------------

If you want to **build** a plugin rather than just use one, you also need
``bsk-sdk``:

.. code-block:: bash

    pip install bsk-sdk

``bsk-sdk`` version numbers track Basilisk — ``bsk-sdk==2.9.1`` contains
headers from Basilisk ``v2.9.1``.  See :ref:`writingPlugins` to get
started building your own plugin.
