from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import spiceInterface

from Basilisk import __path__
bskPath = __path__[0]


def createOneSim():
    """
    Create a minimal Basilisk simulation containing a single SpiceInterface.

    Returns
    -------
    TotalSim : Basilisk SimulationBaseClass instance
        The newly created simulation object.
    SpiceObject : SpiceInterface
        The SpiceInterface instance inside the created simulation.
    """
    TotalSim = SimulationBaseClass.SimBaseClass()
    DynUnitTestProc = TotalSim.CreateNewProcess("process")
    DynUnitTestProc.addTask(TotalSim.CreateNewTask("task", 1))

    # Create and register the SpiceInterface
    SpiceObject = spiceInterface.SpiceInterface()
    SpiceObject.SPICEDataPath = bskPath + '/supportData/EphemerisData/'
    TotalSim.AddModelToTask("task", SpiceObject)

    # Run long enough for the SpiceInterface to furnish its kernels
    TotalSim.ConfigureStopTime(2)
    TotalSim.InitializeSimulation()
    TotalSim.ExecuteSimulation()

    return TotalSim, SpiceObject


def test_multipleInterfaces():
    """
    Verify that SPICE kernels loaded through SpiceInterface are correctly
    reference-counted and unloaded when all SpiceInterface instances are gone.

    The test performs the following high-level checks:

    1. Before creating any SpiceInterface objects, the target kernel must not
       be loaded in SPICE.

    2. Creating the first simulation should cause the kernel to be furnished.

    3. Creating many additional simulations must *not* load the kernel again.
       SPICE can only ever load 5000 kernels, so if we can load > 5000 simulations
       it means that we're not reloading kernels unnecessarily.

    4. After all simulations fall out of scope and Python's garbage collector
       runs, the kernel must be fully unloaded from SPICE.

    This guarantees that:
      - furnsh_c() is only called once per unique kernel file
      - unload_c() is only called when the last user disappears
      - the shared-pointer-based lifetime system works correctly
    """
    kernel = f"{bskPath}/supportData/EphemerisData/de430.bsp"

    # Step 1 - Kernel not yet loaded
    assert not spiceInterface.isKernelLoaded(kernel)

    def smallScope():
        # Step 2 - First SpiceInterface loads the kernel
        firstSim, firstSpice = createOneSim()
        assert spiceInterface.isKernelLoaded(kernel)

        # Step 3 - Many more SpiceInterfaces do NOT reload the kernel
        cacheSims = []
        N = 5005
        for _ in range(N):
            cacheSims.append(createOneSim())

        # Still loaded exactly once
        assert spiceInterface.isKernelLoaded(kernel)

    # Everything in smallScope is destroyed once we leave the function
    smallScope()

    # Force Python to release all cached SpiceInterface objects
    # just in case, probably not needed with CPython
    import gc
    gc.collect()

    # Step 4 - Kernel must now be fully unloaded
    assert not spiceInterface.isKernelLoaded(kernel)


if __name__ == "__main__":
    test_multipleInterfaces()
