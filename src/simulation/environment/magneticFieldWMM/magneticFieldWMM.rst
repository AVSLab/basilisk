Executive Summary
-----------------

Evaluate a magnetic field model about Earth for a range of spacecraft locations using the World Magnetic Model (WMM).

For more information on this module see this For more information on this module see this :download:`PDF Description </../../src/simulation/environment/magneticFieldWMM/_Documentation/Basilisk-magFieldWMM-20190618.pdf>`.

The module is a sub-class of the :ref:`magneticFieldBase` base class.  See that class for the nominal messages
used and general instructions.

User Guide
----------
The :ref:`MagneticFieldWMM` model is created using:

.. code-block:: python
    :linenos:

    from Basilisk.utilities.supportDataTools.dataFetcher import get_path, DataFile
    from Basilisk.simulation import magneticFieldWMM

    magModule = magneticFieldWMM.MagneticFieldWMM()
    magModule.ModelTag = "WMM"

    # Configuration of WMM coefficient file
    wmm_file = get_path(DataFile.MagneticFieldData.WMM)
    magModule.configureWMMFile(str(wmm_file))

    scSim.AddModelToTask(dynTaskName, magModule)
