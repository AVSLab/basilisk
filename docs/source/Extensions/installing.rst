.. toctree::
   :maxdepth: 1
   :hidden:

.. _extensionsInstall:

Installing Extensions
=====================

A Basilisk extension is a standard Python wheel.  Installing one requires
only ``pip`` — no compiler, no source checkout, no rebuilding Basilisk.

Installing an Extension
-----------------------

.. code-block:: bash

    pip install <extension-name>

For example, if an extension is published to PyPI as ``my-atm-extension``:

.. code-block:: bash

    pip install my-atm-extension

This installs the extension and pulls in Basilisk as a dependency
automatically.  To ensure optional Basilisk components are available, install
the recommended Basilisk extra explicitly:

.. code-block:: bash

    pip install "bsk[all]" <extension-name>

Once installed, use the extension like any built-in Basilisk module:

.. code-block:: python

    from my_atm_extension import customAtmosphere
    from Basilisk.utilities import SimulationBaseClass

    sim = SimulationBaseClass.SimBaseClass()
    atm = customAtmosphere.CustomAtmosphere()
    sim.AddModelToTask("task", atm)

Version Compatibility
---------------------

An extension wheel is compiled against a specific Basilisk version.  The
extension's own version number is controlled by the extension developer and does
not need to match Basilisk's version.  What matters is that the wheel was built
against the same BSK headers as the Basilisk release you have installed.

Always verify you have the correct Basilisk version before installing an
extension.  The extension's documentation should state which Basilisk version it
targets:

.. code-block:: bash

    pip install "bsk[all]==2.11.0" "my-extension"

If the installed Basilisk version does not match what the extension was compiled
against, CMake will error at configure time with an error message.

.. note::
   An extension built with ``bsk-sdk==2.X.Y`` only works with Basilisk ``2.X.Y``.
   Upgrading Basilisk means rebuilding the extension against the matching SDK
   version. We are exploring ways to make version compatibility easier in the
   future.

Installing the SDK (for extension authors)
------------------------------------------

If you want to **build** an extension rather than just use one, you also need
``bsk-sdk``:

.. code-block:: bash

    pip install bsk-sdk

``bsk-sdk`` version numbers track Basilisk — ``bsk-sdk==2.11.0`` contains
headers from Basilisk ``v2.11.0``.  See :ref:`writingExtensions` to get
started building your own extension.
