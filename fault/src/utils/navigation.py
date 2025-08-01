from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import simpleNav
from Basilisk.fswAlgorithms import attTrackingError, mrpFeedback, inertial3D


def setup_navigation_and_control(scSim: SimulationBaseClass.SimBaseClass, simTaskName: str):
    """
    Sets up the navigation and control FSW modules and adds them to the simulation task.

    Args:
        scSim: The simulation base class instance.
        simTaskName: The name of the simulation task.

    Returns:
        A dictionary containing the added modules:
            - "sNav": SimpleNav object
            - "inertial3D": inertial3D guidance module
            - "attError": attitude tracking error module
            - "mrpControl": MRP feedback control module
    """
    # --- Setup Navigation Sensor (SimpleNav) ---
    sNavObject = simpleNav.SimpleNav()
    sNavObject.ModelTag = "SimpleNavigation"
    scSim.AddModelToTask(simTaskName, sNavObject)

    # --- Inertial3D Guidance ---
    inertial3DObj = inertial3D.inertial3D()
    inertial3DObj.ModelTag = "inertial3D"
    inertial3DObj.sigma_R0N = [0.0, 0.0, 0.0]  # Desired inertial orientation
    scSim.AddModelToTask(simTaskName, inertial3DObj)

    # --- Attitude Tracking Error ---
    attError = attTrackingError.attTrackingError()
    attError.ModelTag = "attErrorInertial3D"
    scSim.AddModelToTask(simTaskName, attError)

    # --- MRP Feedback Control ---
    mrpControl = mrpFeedback.mrpFeedback()
    mrpControl.ModelTag = "mrpFeedback"
    mrpControl.K = 3.5
    mrpControl.Ki = -1.0  # Negative disables integral control
    mrpControl.P = 30.0
    mrpControl.integralLimit = 2.0 / abs(mrpControl.Ki) * 0.1
    scSim.AddModelToTask(simTaskName, mrpControl)

    return sNavObject, inertial3DObj, attError, mrpControl
