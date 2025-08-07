from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion, unitTestSupport
from Basilisk.simulation import spacecraft
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import (macros,simIncludeRW, unitTestSupport)
from Basilisk.architecture import messaging
from Basilisk.simulation import reactionWheelStateEffector

def setup_spacecraft_sim(true_mode=0, simTimeSec=600, simTimeStepSec=0.1):
    # --- Create Simulation ---
    simTaskName = "simTask"
    simProcessName = "simProcess"
    scSim = SimulationBaseClass.SimBaseClass()
    simulationTime = macros.sec2nano(simTimeSec)
    simulationTimeStep = macros.sec2nano(simTimeStepSec)

    dynProcess = scSim.CreateNewProcess(simProcessName)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # --- Setup Gravity ---
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    mu = earth.mu

    # --- Create Spacecraft ---
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"

    I = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject.hub.mHub = 750.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    scObject.hub.r_CN_NInit = rN
    scObject.hub.v_CN_NInit = vN
    scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    scSim.AddModelToTask(simTaskName, scObject, 1)
    gravFactory.addBodiesTo(scObject)

    # --- Setup Reaction Wheels ---
    # make a fresh RW factory instance, this is critical to run multiple times
    rwFactory = simIncludeRW.rwFactory()
    varRWModel = messaging.BalancedWheels

    # Create RWs
    rw_List = []
    for i, gsHat in enumerate([[1, 0, 0], [0, 1, 0], [0, 0, 1]]):
        # Assign initial Omega (RPM)
        omega_init = 100. + 100. * i

        # Optional position vector for RW3
        rWB_B = [0.5, 0.5, 0.5] if i == 2 else None

        # Create the RW with appropriate parameters
        if rWB_B is not None:
            rw = rwFactory.create(
                'Honeywell_HR16',
                gsHat,
                maxMomentum=50.,
                Omega=omega_init,
                RWModel=varRWModel,
                rWB_B=rWB_B
            )
        else:
            rw = rwFactory.create(
                'Honeywell_HR16',
                gsHat,
                maxMomentum=50.,
                Omega=omega_init,
                RWModel=varRWModel,
            )
        rw_List.append(rw)
    # Inject fault in the one matching true_mode
    if(true_mode == 1):
        rw_List[0].Js *= 0.5

    if(true_mode == 2):
        rw_List[1].Js *= 0.5

    if(true_mode == 3):
        rw_List[2].Js *= 0.5
        
    if(true_mode == -1):
        rw_List[0].Js *= 0.5
        rw_List[1].Js *= 0.5
        rw_List[2].Js *= 0.5

    numRW = rwFactory.getNumOfDevices()

    # --- Connect Reaction Wheels to Spacecraft via rwStateEffector ---
    rwStateEffector = reactionWheelStateEffector.ReactionWheelStateEffector()
    rwStateEffector.ModelTag = "RW_cluster"
    rwFactory.addToSpacecraft(scObject.ModelTag, rwStateEffector, scObject)
    scSim.AddModelToTask(simTaskName, rwStateEffector, 2)

    return (
        scSim,
        scObject,
        simTaskName,
        simTimeSec,
        simTimeStepSec,
        simulationTime,
        simulationTimeStep,
        varRWModel,
        rwFactory,
        rwStateEffector,
        numRW,
        I,
    )
