from pathlib import Path

from Basilisk import __path__
from Basilisk.simulation import spiceInterface
from Basilisk.utilities import SimulationBaseClass

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
    SpiceObject.SPICEDataPath = str(Path(bskPath) / "supportData" / "EphemerisData")
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

    4. After all simulations have loaded, the number of loaded kernels should
       be the same as after loading one sim.

    5. After all simulations fall out of scope and Python's garbage collector
       runs, the kernel must be fully unloaded from SPICE.

    This guarantees that:
      - furnsh_c() is only called once per unique kernel file
      - unload_c() is only called when the last user disappears
      - the shared-pointer-based lifetime system works correctly
    """

    # Step 1: nothing loaded yet
    assert spiceInterface.countKernelsLoaded() == 0

    def smallScope():
        # Step 2: first sim loads (pooch or local)
        firstSim, firstSpice = createOneSim()
        kernelsLoadedWithOneSim = spiceInterface.countKernelsLoaded()
        assert kernelsLoadedWithOneSim > 0

        # Step 3: many more sims do not increase kernel count
        cacheSims = []
        for _ in range(20):
            cacheSims.append(createOneSim())

        kernelsLoadedWithNSims = spiceInterface.countKernelsLoaded()
        assert kernelsLoadedWithOneSim == kernelsLoadedWithNSims

    smallScope()

    import gc

    gc.collect()

    # Step 5: everything should be unloaded
    assert spiceInterface.countKernelsLoaded() == 0


if __name__ == "__main__":
    test_multipleInterfaces()
