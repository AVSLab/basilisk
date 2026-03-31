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

A plugin wheel is compiled against a specific Basilisk version.  The plugin's
own version number is controlled by the plugin developer and does not need to
match Basilisk's version.  What matters is that the wheel was built against the
same BSK headers as the Basilisk release you have installed.

Always verify you have the correct Basilisk version before installing a plugin.
The plugin's documentation should state which Basilisk version it targets:

.. code-block:: bash

    pip install "bsk==2.9.1" "my-plugin"

If the installed Basilisk version does not match what the plugin was compiled
against, CMake will error at configure time with an error message.

.. note::
   A plugin built with ``bsk-sdk==2.X.Y`` only works with Basilisk ``2.X.Y``.
   Upgrading Basilisk means rebuilding the plugin against the matching SDK
   version. We are exploring ways to make version compatibility easier in the
   future.

Installing the SDK (for plugin authors)
----------------------------------------

If you want to **build** a plugin rather than just use one, you also need
``bsk-sdk``:

.. code-block:: bash

    pip install bsk-sdk

``bsk-sdk`` version numbers track Basilisk — ``bsk-sdk==2.9.1`` contains
headers from Basilisk ``v2.9.1``.  See :ref:`writingPlugins` to get
started building your own plugin.
