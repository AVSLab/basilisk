import os

import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
from Basilisk.simulation import facetDragDynamicEffector
# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.simulation import tabularAtmosphere, simpleNav
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import vizSupport
from Basilisk.utilities.readAtmTable import readAtmTable
from Basilisk.simulation import hingedRigidBodyStateEffector
from Basilisk.simulation import dualHingedRigidBodyStateEffector
from Basilisk.utilities import RigidBodyKinematics as rbk

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

def sph2rv(xxsph):
    """
    NOTE: this function assumes inertial and planet-fixed frames are aligned
    at this time
    """
    
    r = xxsph[0]
    lon = xxsph[1]
    lat = xxsph[2]
    u = xxsph[3]
    gam = xxsph[4]
    hda = xxsph[5]
    
    NI = np.eye(3)
    IE = np.array([[np.cos(lat) * np.cos(lon), -np.sin(lon), -np.sin(lat) * np.cos(lon)],
                   [np.cos(lat) * np.sin(lon), np.cos(lon), -np.sin(lat) * np.sin(lon)],
                   [np.sin(lat), 0, np.cos(lat)]])
    ES = np.array([[np.cos(gam), 0, np.sin(gam)],
                   [-np.sin(gam) * np.sin(hda), np.cos(hda), np.cos(gam) * np.sin(hda)],
                   [-np.sin(gam) * np.cos(hda), -np.sin(hda), np.cos(gam) * np.cos(hda)]])
    
    e1_E = np.array([1,0,0])
    rvec_N = (r * NI @ IE) @ e1_E
    
    s3_S = np.array([0,0,1])
    uvec_N = u * ( NI @ IE @ ES) @ s3_S
    
    return rvec_N, uvec_N


def run(planetCase, DOF, deorbitAlt):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        planetCase (string): Specify if a `Mars` or `Earth` arrival is simulated

    """

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.01)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # Construct algorithm and associated C++ container
    # change module to tabAtmo
    tabAtmo = tabularAtmosphere.TabularAtmosphere()   # update with current values
    tabAtmo.ModelTag = "tabularAtmosphere"            # update python name of test module
    atmoTaskName = "atmosphere"
    
    # define constants & load data
    if planetCase == 'Earth':
        r_eq = 6378136.6
        dataFileName = bskPath + '/supportData/AtmosphereData/EarthGRAMNominal.txt'
        altList, rhoList, tempList = readAtmTable(dataFileName, 'EarthGRAM')
    else:
        r_eq = 3397.2 * 1000
        dataFileName = bskPath + '/supportData/AtmosphereData/MarsGRAMNominal.txt'
        altList, rhoList, tempList = readAtmTable(dataFileName, 'MarsGRAM')
        
    # assign constants & ref. data to module
    tabAtmo.planetRadius = r_eq
    tabAtmo.altList = tabularAtmosphere.DoubleVector(altList)    
    tabAtmo.rhoList = tabularAtmosphere.DoubleVector(rhoList)
    tabAtmo.tempList = tabularAtmosphere.DoubleVector(tempList)

    # Drag Effector for sc1 
    drag1 = facetDragDynamicEffector.FacetDragDynamicEffector()
    drag1.ModelTag = "FacetDrag1"
    dragEffectorTaskName1 = "drag1" #should there be one task name per drag1 and drag 2?
    scAreas = 25.0
    scCoeff = 2.0
    
    B_normals = [
        np.array([ 1, 0, 0]), 
        np.array([-1, 0, 0]), 
        np.array([ 0, 1, 0]), 
        np.array([ 0, -1, 0]), 
        np.array([ 0, 0, 1]), 
        np.array([ 0, 0, -1]) 
    ]

    B_locations = [
        np.array([ 0.2, 0.0, 0.0]),
        np.array([-0.2, 0.0, 0.0]),
        np.array([ 0.0, 0.2, 0.0]),
        np.array([ 0.0, -0.2, 0.0]),
        np.array([ 0.0, 0.0, 0.2]),
        np.array([ 0.0, 0.0, -0.2])
    ]
    
    for ind in range(0,len(B_normals)):
        drag1.addFacet(scAreas, scCoeff, B_normals[ind], B_locations[ind])

    # Drag Effectors for sc2
    drag2 = facetDragDynamicEffector.FacetDragDynamicEffector()
    drag2.ModelTag = "FacetDrag2"
    dragEffectorTaskName2 = "drag2"

    for ind in range(0,len(B_normals)):
        drag2.addFacet(scAreas, scCoeff, B_normals[ind], B_locations[ind])

    drag3 = facetDragDynamicEffector.FacetDragDynamicEffector()
    drag3.ModelTag = "FacetDrag3"
    dragEffectorTaskName3 = "panelDrag3"

    drag4 = facetDragDynamicEffector.FacetDragDynamicEffector()
    drag4.ModelTag = "FacetDrag4"
    dragEffectorTaskName4 = "panelDrag4"
    
    panelArea = 10    # m^2 ???
    panelOffset = 0.3  # Distance from hub COM CHECK THIS!!!!!!!!!
    panelCd = 5

    panel_normals = [
        np.array([0, 0,  1]), 
        np.array([0, 0, -1])
    ]

    # Panel 1 locations (at +0.5 in x)
    panel1_locations = [
        np.array([0.5, 0.0, panelOffset]),  # Facet pointing +z
        np.array([0.5, 0.0, panelOffset])   # Facet pointing -z
    ]

    # Panel 3 locations (at -0.5 in x)
    panel3_locations = [
        np.array([-0.5, 0.0, panelOffset]),  # Facet pointing +z
        np.array([-0.5, 0.0, panelOffset])   # Facet pointing -z
    ]

    panel2_locations = panel1_locations

    panel4_locations = panel3_locations

    for i in range(2):
        drag3.addFacet(panelArea, panelCd, panel_normals[i], panel2_locations[i])

    for i in range(2):
        drag4.addFacet(panelArea, panelCd, panel_normals[i], panel4_locations[i])
    
    for i in range(2):
        drag1.addFacet(panelArea, panelCd, panel_normals[i], panel1_locations[i])
    for i in range(2):
        drag1.addFacet(panelArea, panelCd, panel_normals[i], panel3_locations[i]) #loc may be diff

    dynProcess.addTask(scSim.CreateNewTask(atmoTaskName, simulationTimeStep))
    
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName1, simulationTimeStep))
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName2, simulationTimeStep))
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName3, simulationTimeStep))
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName4, simulationTimeStep))
    
    scSim.AddModelToTask(atmoTaskName, tabAtmo)

    scSim.AddModelToTask(simTaskName, tabAtmo)

    #
    #   setup the simulation tasks/objects
    #

    m_sc = 2600.0  # kg

    # initialize spacecraft object and set properties
    scObject1 = spacecraft.Spacecraft() # panel fixed over time from POV of drag 
    scObject1.ModelTag = "spacecraftBody1"
    scObject1.hub.mHub = m_sc
    scObject1.hub.IHubPntBc_B = [[1500.0, 0.0, 0.0], [0.0, 1500.0, 0.0], [0.0, 0.0, 2000.0]]
    tabAtmo.addSpacecraftToModel(scObject1.scStateOutMsg)

    scObject2 = spacecraft.Spacecraft() # one facited drag to hub, another two for panels
    scObject2.ModelTag = "spacecraftBody2"
    scObject2.hub.mHub = m_sc
    scObject2.hub.IHubPntBc_B = [[1500.0, 0.0, 0.0], [0.0, 1500.0, 0.0], [0.0, 0.0, 2000.0]]
    tabAtmo.addSpacecraftToModel(scObject2.scStateOutMsg)
    
    simpleNavObj = simpleNav.SimpleNav()
    scSim.AddModelToTask(simTaskName, simpleNavObj)
    simpleNavObj.scStateInMsg.subscribeTo(scObject1.scStateOutMsg)
    simpleNavObj.scStateInMsg.subscribeTo(scObject2.scStateOutMsg)

    # Panel 1 is not the parent to the drag effector 
    panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector() 
    panel1.ModelTag = "panel1"

    # Panel 2 is the parent to the drag effector (and so is the hub)
    panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    panel2.ModelTag = "panel2"

    panel2DOF_1 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    panel2DOF_1.ModelTag = "panel1"

    panel2DOF_2 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    panel2DOF_2.ModelTag = "panel2"

    # panelNDOF_1 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    # panelNDOF_1.ModelTag = "panel1"

    # panelNDOF_2 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    # panelNDOF_2.ModelTag = "panel2"

    # All panels are identical exept for location 
    if DOF == "1": 
        panelMass = 10
    else: 
        panelMass = 5
    I_realistic = (1/12) * panelMass * (3.0**2 + 2.0**2)
    IPntS_S = [[I_realistic, 0.0, 0.0], [0.0, I_realistic, 0.0], [0.0, 0.0, I_realistic]]
    k = 5000
    c_critical = 2 * np.sqrt(k * 25) 
    c = 2.0 * c_critical # Overdamped system
    d = 1.5
    dcm_HB = [[1, 0, 0.0], [0.0, 1, 0.0], [0.0, 0.0, 1]]

    #1DOF
    panel1.mass = panelMass
    panel1.IPntS_S = IPntS_S
    panel1.d = -d
    panel1.k = k
    panel1.c = c  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    panel1.dcm_HB = dcm_HB
    panel1.thetaInit = 0.0
    panel1.thetaDotInit = 0.0

    panel2.mass = panelMass
    panel2.IPntS_S = IPntS_S
    panel2.d =-d
    panel2.k = k
    panel2.c = c  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    panel2.r_HB_B = [[0.5], [0.0], [1.0]] 
    panel2.dcm_HB = dcm_HB
    panel2.thetaInit = 0.0
    panel2.thetaDotInit = 0.0

    # 2DOF 
    panel2DOF_1.mass1 = panelMass
    panel2DOF_1.mass2 = panelMass
    panel2DOF_1.d1 = -d 
    panel2DOF_1.d2 = -d
    panel2DOF_1.k1 = k/2
    panel2DOF_1.k2 = k/2
    panel2DOF_1.c1 = c/2
    panel2DOF_1.c2 = c/2
    panel2DOF_1.l1 = -(2*d)
    panel2DOF_1.r_H1B_B = [[0.5], [0.0], [1.0]]
    panel2DOF_1.dcm_H1B = dcm_HB
    panel2DOF_1.theta1Init = 0.0
    panel2DOF_1.theta1DotInit = 0.0
    panel2DOF_1.IPntS2_S2 = IPntS_S
    panel2DOF_1.theta2Init = 0.0
    panel2DOF_1.theta2DotInit = 0.0

    panel2DOF_2.mass1 = panelMass
    panel2DOF_2.mass2 = panelMass
    panel2DOF_2.d1 = -d 
    panel2DOF_2.d2 = -d
    panel2DOF_2.k1 = k/2
    panel2DOF_2.k2 = k/2
    panel2DOF_2.c1 = c/2
    panel2DOF_2.c2 = c/2
    panel2DOF_2.l1 = -(2*d)
    panel2DOF_2.r_H1B_B = [[0.5], [0.0], [1.0]]
    panel2DOF_2.dcm_H1B = dcm_HB
    panel2DOF_2.theta1Init = 0.0
    panel2DOF_2.theta1DotInit = 0.0
    panel2DOF_2.IPntS2_S2 = IPntS_S
    panel2DOF_2.theta2Init = 0.0
    panel2DOF_2.theta2DotInit = 0.0

    # # Symmetrically opposite panels for each spacecraft
    panel3 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector() 
    panel3.ModelTag = "panel3"

    panel4 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector() 
    panel4.ModelTag = "panel4"

    panel2DOF_3 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    panel2DOF_3.ModelTag = "panel3"

    panel2DOF_4 = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    panel2DOF_4.ModelTag = "panel4"

    # 1DOF
    panel3.mass = panelMass
    panel3.IPntS_S = IPntS_S
    panel3.d = d
    panel3.k = k
    panel3.c = c  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    panel3.r_HB_B = [[-0.5], [0.0], [1.0]]
    panel3.dcm_HB = dcm_HB
    panel3.thetaInit = 0.0
    panel3.thetaDotInit = 0.0

    panel4.mass = panelMass
    panel4.IPntS_S = IPntS_S
    panel4.d = d
    panel4.k = k
    panel4.c = c  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    panel4.r_HB_B = [[-0.5], [0.0], [1.0]] 
    panel4.dcm_HB = dcm_HB
    panel4.thetaInit = 0.0
    panel4.thetaDotInit = 0.0

    # 2DOF 
    panel2DOF_3.mass1 = panelMass
    panel2DOF_3.mass2 = panelMass
    panel2DOF_3.d1 = d
    panel2DOF_3.d2 = d
    panel2DOF_3.k1 = k/2
    panel2DOF_3.k2 = k/2
    panel2DOF_3.c1 = c/2
    panel2DOF_3.c2 = c/2
    panel2DOF_3.l1 = (2*d)
    panel2DOF_3.r_H1B_B = [[-0.5], [0.0], [1.0]]
    panel2DOF_3.dcm_H1B = dcm_HB
    panel2DOF_3.theta1Init = 0.0
    panel2DOF_3.theta1DotInit = 0.0
    panel2DOF_3.IPntS2_S2 = IPntS_S
    panel2DOF_3.theta2Init = 0.0
    panel2DOF_3.theta2DotInit = 0.0

    panel2DOF_4.mass1 = panelMass
    panel2DOF_4.mass2 = panelMass
    panel2DOF_4.d1 = d
    panel2DOF_4.d2 = d
    panel2DOF_4.k1 = k/2
    panel2DOF_4.k2 = k/2
    panel2DOF_4.c1 = c/2
    panel2DOF_4.c2 = c/2
    panel2DOF_4.l1 = (2*d)
    panel2DOF_4.r_H1B_B = [[-0.5], [0.0], [1.0]]
    panel2DOF_4.dcm_H1B = dcm_HB
    panel2DOF_4.theta1Init = 0.0
    panel2DOF_4.theta1DotInit = 0.0
    panel2DOF_4.IPntS2_S2 = IPntS_S
    panel2DOF_4.theta2Init = 0.0
    panel2DOF_4.theta2DotInit = 0.0

    #########
    # Add panels to spaceCraft
    #########

    # in order to affect dynamics

    if DOF == "1": 
        scObject1.addStateEffector(panel1)
        scObject2.addStateEffector(panel2)
        scObject1.addStateEffector(panel3)
        scObject2.addStateEffector(panel4)

        scSim.AddModelToTask(simTaskName, scObject1)
        scSim.AddModelToTask(simTaskName, scObject2)

        # in order to track messages
        scSim.AddModelToTask(simTaskName, panel1) 
        scSim.AddModelToTask(simTaskName, panel2)
        scSim.AddModelToTask(simTaskName, panel3)
        scSim.AddModelToTask(simTaskName, panel4)
    elif DOF == "2": 
        scObject1.addStateEffector(panel2DOF_1)
        scObject2.addStateEffector(panel2DOF_2)
        scObject1.addStateEffector(panel2DOF_3)
        scObject2.addStateEffector(panel2DOF_4)

        scSim.AddModelToTask(simTaskName, scObject1)
        scSim.AddModelToTask(simTaskName, scObject2)

        # in order to track messages
        scSim.AddModelToTask(simTaskName, panel2DOF_1) 
        scSim.AddModelToTask(simTaskName, panel2DOF_2)
        scSim.AddModelToTask(simTaskName, panel2DOF_3)
        scSim.AddModelToTask(simTaskName, panel2DOF_4)

    
    # Attach drag to hub only (previous method)
    scObject1.addDynamicEffector(drag1) # remember later to add the other panel facets to the drag!!!
    scSim.AddModelToTask(dragEffectorTaskName1, drag1)
    drag1.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])
    
    # Attach drag to hub and panel (new/more accurate model)
    scObject2.addDynamicEffector(drag2)
    scSim.AddModelToTask(dragEffectorTaskName2, drag2)
    drag2.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])

    if DOF == "1": 
        panel2.addDynamicEffector(drag3)  
    elif DOF == "2": 
        panel2DOF_2.addDynamicEffector(drag3,1)
    scSim.AddModelToTask(dragEffectorTaskName3, drag3)
    drag3.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])    

    if DOF == "1":
        panel4.addDynamicEffector(drag4)  
    elif DOF == "2":
        panel2DOF_4.addDynamicEffector(drag4,1)   # need to make 2 more drags for the other 2 panels that are atached to the first panel
    scSim.AddModelToTask(dragEffectorTaskName4, drag4)
    drag4.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])    

    # set the simulation time
    if planetCase == 'Earth':
        simulationTime = macros.sec2nano(9600)
    else:
        simulationTime = macros.sec2nano(11600)

    # Setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createBody(planetCase)
    planet.isCentralBody = True  # ensure this is the central gravitational body

    # Attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject1)
    gravFactory.addBodiesTo(scObject2)

    if planetCase == 'Earth':
        r = 6503 * 1000.
        u = 10.5 * 1000 # 11 km/s is the escape velocity 
        gam = -5.15 * macros.D2R
    else:
        r = (3397.2 + 125.) * 1000
        u = 4.5 * 1000 #5.03 km/s is escape velocity
        gam = -10 * macros.D2R
    lon = 0
    lat = 0
    hda = np.pi/2
    xxsph = [r,lon,lat,u,gam,hda]
    rN, vN = sph2rv(xxsph)
    
    scObject1.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject1.hub.v_CN_NInit = vN  # m - v_CN_N
    scObject2.hub.r_CN_NInit = rN
    scObject2.hub.v_CN_NInit = vN

    ####################

    phi_x = 90.0 * macros.D2R
    phi_z = 90.0 * macros.D2R
    C_x = rbk.euler1212C([phi_x, 0.0, 0.0]) # rotation about x
    C_z = rbk.euler3132C([phi_z, 0.0, 0.0]) # rotation about z  
    phi_final = 5.0 * macros.D2R  # Final trim

    # Apply x first, then z
    C_BN_base = C_z @ C_x

    C_trim = rbk.euler1212C([phi_final, 0.0, 0.0])

    C_BN = C_trim @ C_BN_base

    sigma_BN = rbk.C2MRP(C_BN)

    scObject1.hub.sigma_BNInit = sigma_BN
    scObject2.hub.sigma_BNInit = sigma_BN

    omega_BN_BInit = [[0.0], [0.0], [0.0]] # rad/s - omega_BN_B

    scObject1.hub.omega_BN_BInit = omega_BN_BInit
    scObject2.hub.omega_BN_BInit = omega_BN_BInit

    ####################

    # # Build velocity-aligned attitude
    # # Body frame: x=velocity (ram), y=orbit normal, z=nadir
    # v_hat = vN / np.linalg.norm(vN)
    # r_hat = rN / np.linalg.norm(rN)

    # b_x = v_hat               # Along velocity
    # b_z = -r_hat              # Toward planet (nadir)
    # b_y = np.cross(b_z, b_x)  # Orbit normal
    # b_y = b_y / np.linalg.norm(b_y)
    # b_z = np.cross(b_x, b_y)  # Recompute for orthogonality

    # # Create DCM (Direction Cosine Matrix)
    # C_BN = np.column_stack([b_x, b_y, b_z])
    # sigma_BN = rbk.C2MRP(C_BN)

    # # Calculate mean orbital motion to maintain alignment
    # mu = planet.mu
    # v_mag = np.linalg.norm(vN)
    # r_mag = np.linalg.norm(rN)

    # specific_energy = 0.5 * v_mag**2 - mu / r_mag
    # a = -mu / (2 * specific_energy)  # Semi-major axis

    # period = 2 * np.pi * np.sqrt(a**3 / mu)
    # omega_magnitude = 2 * np.pi / period

    # h_vec = np.cross(rN, vN)  # Angular momentum vector
    # h_hat = h_vec / np.linalg.norm(h_vec)  # Orbit normal direction
    # omega_orbit_N = omega_magnitude * h_hat  # VECTOR in inertial frame
                                 
    # # Transform to body frame
    # omega_BN_B = C_BN.T @ omega_orbit_N

    # # Set initial conditions
    # scObject1.hub.sigma_BNInit = sigma_BN.reshape(3,1)
    # scObject2.hub.sigma_BNInit = sigma_BN.reshape(3,1)
    # scObject1.hub.omega_BN_BInit = omega_BN_B.reshape(3,1)
    # scObject2.hub.omega_BN_BInit = omega_BN_B.reshape(3,1)

    ####################

    #
    #   Setup data logging before the simulation is initialized
    #

    dataLog1 = scObject1.scStateOutMsg.recorder()
    dataLog2 = scObject2.scStateOutMsg.recorder()

    if DOF == "1":
        p1Log = panel1.hingedRigidBodyOutMsg.recorder()
        p2Log = panel2.hingedRigidBodyOutMsg.recorder()
        p3Log = panel3.hingedRigidBodyOutMsg.recorder()
        p4Log = panel4.hingedRigidBodyOutMsg.recorder()
    elif DOF == "2": 
        p1Log = panel2DOF_1.dualHingedRigidBodyOutMsgs[0].recorder()
        p2Log = panel2DOF_1.dualHingedRigidBodyOutMsgs[1].recorder()

        p3Log = panel2DOF_2.dualHingedRigidBodyOutMsgs[0].recorder()
        p4Log = panel2DOF_2.dualHingedRigidBodyOutMsgs[1].recorder()

        p5Log = panel2DOF_3.dualHingedRigidBodyOutMsgs[0].recorder()
        p6Log = panel2DOF_3.dualHingedRigidBodyOutMsgs[1].recorder()

        p7Log = panel2DOF_4.dualHingedRigidBodyOutMsgs[0].recorder()
        p8Log = panel2DOF_4.dualHingedRigidBodyOutMsgs[1].recorder()

    dataNewAtmoLog = tabAtmo.envOutMsgs[0].recorder()

    scSim.AddModelToTask(simTaskName, dataLog1)
    scSim.AddModelToTask(simTaskName, dataLog2)
    scSim.AddModelToTask(simTaskName, p1Log)
    scSim.AddModelToTask(simTaskName, p2Log)
    scSim.AddModelToTask(simTaskName, p3Log)
    scSim.AddModelToTask(simTaskName, p4Log)
    if DOF == "2":
        scSim.AddModelToTask(simTaskName, p5Log)
        scSim.AddModelToTask(simTaskName, p6Log)
        scSim.AddModelToTask(simTaskName, p7Log)
        scSim.AddModelToTask(simTaskName, p8Log)

    scSim.AddModelToTask(simTaskName, dataNewAtmoLog)

    dataNewAtmoLog = tabAtmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask(simTaskName, dataNewAtmoLog)

    # Event to terminate the simulation
    scSim.createNewEvent(
        "Deorbited",
        simulationTimeStep,
        True,
        conditionFunction=lambda self: (
            np.linalg.norm(scObject1.scStateOutMsg.read().r_BN_N)
            < planet.radEquator + 1000 * deorbitAlt
        or 
            np.linalg.norm(scObject2.scStateOutMsg.read().r_BN_N)
            < planet.radEquator + 1000 * deorbitAlt
        ),
        terminal=True,
    )

    if DOF == "1": 
        scBodyList = [
            scObject1,
            ["panel1", panel1.hingedRigidBodyConfigLogOutMsg],
            ["panel3", panel3.hingedRigidBodyConfigLogOutMsg],
            scObject2,
            ["panel2", panel2.hingedRigidBodyConfigLogOutMsg],
            ["panel4", panel4.hingedRigidBodyConfigLogOutMsg],
        ]
    elif DOF == "2": 
        scBodyList = [
            scObject1,
            ["panel1_seg1", panel2DOF_1.dualHingedRigidBodyConfigLogOutMsgs[0]],
            ["panel1_seg2", panel2DOF_1.dualHingedRigidBodyConfigLogOutMsgs[1]],
            ["panel3_seg1", panel2DOF_3.dualHingedRigidBodyConfigLogOutMsgs[0]],
            ["panel3_seg2", panel2DOF_3.dualHingedRigidBodyConfigLogOutMsgs[1]],
            scObject2,
            ["panel2_seg1", panel2DOF_2.dualHingedRigidBodyConfigLogOutMsgs[0]],
            ["panel2_seg2", panel2DOF_2.dualHingedRigidBodyConfigLogOutMsgs[1]],
            ["panel4_seg1", panel2DOF_4.dualHingedRigidBodyConfigLogOutMsgs[0]],
            ["panel4_seg2", panel2DOF_4.dualHingedRigidBodyConfigLogOutMsgs[1]],
        ]

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    viz = vizSupport.enableUnityVisualization(scSim, simTaskName, scBodyList
                                              , saveFile=fileName
                                              )
    # Spacecraft 1 hub
    vizSupport.createCustomModel(viz,
                                    simBodiesToModify=[scObject1.ModelTag],
                                    modelPath="CUBE",
                                    color=vizSupport.toRGBA255("red"),
                                    scale=[1, 2, 2])  # [width, length, height] in meters
    # Spacecraft 2 hub
    vizSupport.createCustomModel(viz,
                                 simBodiesToModify=[scObject2.ModelTag],
                                 modelPath="CUBE",
                                 color=vizSupport.toRGBA255("blue"),
                                 scale=[1, 2, 2])  # [width, length, height] in meters

    if DOF == "1":
        # Panel 1 on spacecraft 1
        vizSupport.createCustomModel(viz,
                                     simBodiesToModify=["panel1"],
                                     modelPath="CUBE",
                                     scale=[3, 2, 0.1]) # [width, length, height] in meters

        # Panel 3 on spacecraft 1
        vizSupport.createCustomModel(viz,
                                     simBodiesToModify=["panel3"],
                                     modelPath="CUBE",
                                     scale=[3, 2, 0.1]) # [width, length, height] in meters

        # Panel 2 on spacecraft 2
        vizSupport.createCustomModel(viz,
                                     simBodiesToModify=["panel2"],
                                     modelPath="CUBE",
                                     color=vizSupport.toRGBA255("gold"),
                                     scale=[3, 2, 0.1])  # [width, length, height] in meters

        # Panel 4 on spacecraft 2
        vizSupport.createCustomModel(viz,
                                     simBodiesToModify=["panel4"],
                                     modelPath="CUBE",
                                     color=vizSupport.toRGBA255("gold"),
                                     scale=[3, 2, 0.1]) # [width, length, height] in meters
    elif DOF == "2":
        # Render both dual-hinge segments for each panel branch.
        for panelName, panelColor in [
            ("panel1_seg1", None),
            ("panel1_seg2", None),
            ("panel3_seg1", None),
            ("panel3_seg2", None),
            ("panel2_seg1", vizSupport.toRGBA255("gold")),
            ("panel2_seg2", vizSupport.toRGBA255("gold")),
            ("panel4_seg1", vizSupport.toRGBA255("gold")),
            ("panel4_seg2", vizSupport.toRGBA255("gold")),
        ]:
            if panelColor is None:
                vizSupport.createCustomModel(viz,
                                             simBodiesToModify=[panelName],
                                             modelPath="CUBE",
                                             scale=[3, 2, 0.1])
                                            
            else:
                vizSupport.createCustomModel(viz,
                                             simBodiesToModify=[panelName],
                                             modelPath="CUBE",
                                             color=panelColor,
                                             scale=[3, 2, 0.1])

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData1 = dataLog1.r_BN_N
    velData1 = dataLog1.v_BN_N
    posData2 = dataLog2.r_BN_N
    velData2 = dataLog2.v_BN_N
    dataOmegaBN_B1 = dataLog1.omega_BN_B
    dataOmegaBN_B2 = dataLog2.omega_BN_B


    panel1thetaLog = p1Log.theta
    panel1thetaDotLog = p1Log.thetaDot
    panel2thetaLog = p2Log.theta
    panel2thetaDotLog = p2Log.thetaDot

    panel3thetaLog = p3Log.theta
    panel3thetaDotLog = p3Log.thetaDot
    panel4thetaLog = p4Log.theta
    panel4thetaDotLog = p4Log.thetaDot

    if DOF == "2":
        panel5thetaLog = p5Log.theta
        panel5thetaDotLog = p5Log.thetaDot
        panel6thetaLog = p6Log.theta
        panel6thetaDotLog = p6Log.thetaDot

        panel7thetaLog = p7Log.theta
        panel7thetaDotLog = p7Log.thetaDot
        panel8thetaLog = p8Log.theta
        panel8thetaDotLog = p8Log.thetaDot

    densData = dataNewAtmoLog.neutralDensity

    mu = planet.mu

    v_mag1 = np.linalg.norm(velData1, axis=1) 
    r_mag1 = np.linalg.norm(posData1, axis=1)
    specific_energy1 = 0.5 * v_mag1**2 - mu / r_mag1
    a1 = -mu / (2 * specific_energy1)  # Semi-major axis 
    period1 = 2 * np.pi * np.sqrt(np.abs(a1)**3 / mu)  

    v_mag2 = np.linalg.norm(velData2, axis=1)
    r_mag2 = np.linalg.norm(posData2, axis=1)
    specific_energy2 = 0.5 * v_mag2**2 - mu / r_mag2
    a2 = -mu / (2 * specific_energy2)  # Semi-major axis 
    period2 = 2 * np.pi * np.sqrt(np.abs(a2)**3 / mu)

    np.set_printoptions(precision=16)

    figureList = {}
    plt.close("all")  # clears out plots from earlier test runs

    # draw the inertial position vector components
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(0,3):
        plt.plot(dataLog1.times()*macros.NANO2MIN, posData1[:, idx]/1000.,
                 color=unitTestSupport.getLineColor(idx,3),
                 label='Drag on hub only: $r_{BN,'+str(idx)+'}$')
    for idx in range(0,3):
        plt.plot(dataLog2.times()*macros.NANO2MIN, posData1[:, idx]/1000.,
                 color=unitTestSupport.getLineColor(idx,3),
                 linestyle='--',
                 label='Drag on hub and panel: $r_{BN,'+str(idx)+'}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Inertial Position [km]')

    r1 = np.linalg.norm(posData1, axis=1)
    v1 = np.linalg.norm(velData1, axis=1)

    r2 = np.linalg.norm(posData2, axis=1)
    v2 = np.linalg.norm(velData2, axis=1)

    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(v1/1e3, (r1-r_eq)/1e3, label="Drag on hub only")
    plt.plot(v2/1e3, (r2-r_eq)/1e3, linestyle='--', label="Drag on hub and panel")
    plt.legend(loc='lower right')
    plt.xlabel('Velocity [km/s]')
    plt.ylabel('Altitude [km]')
    plt.grid()
    pltName = fileName + "2" + planetCase
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(dataLog1.times()*macros.NANO2MIN, (r1-r_eq)/1e3, label="Drag on hub only")
    plt.plot(dataLog2.times()*macros.NANO2MIN, (r2-r_eq)/1e3, linestyle='--',label="Drag on hub and panel")
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Altitude [km]')
    plt.grid()
    pltName = fileName + "3" + planetCase
    figureList[pltName] = plt.figure(3)

    # rotating panel hinge angle: panel1 and panel3
    plt.figure(4)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(p1Log.times()*macros.NANO2MIN, panel1thetaLog, label="Panel1")    
    plt.plot(p3Log.times()*macros.NANO2MIN, panel3thetaLog, label="Panel3") 
    plt.legend(loc='lower right')   
    plt.xlabel('Time [min]')    
    plt.ylabel('Panel Angle: drag on hub only [rad]')
    plt.grid()
    pltName = fileName + "4" + planetCase
    figureList[pltName] = plt.figure(4)

    plt.figure(5)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(p2Log.times()*macros.NANO2MIN, panel2thetaLog, label="Panel2")    
    plt.plot(p4Log.times()*macros.NANO2MIN, panel4thetaLog, label="Panel4") 
    plt.legend(loc='lower right')   
    plt.xlabel('Time [min]')    
    plt.ylabel('Panel Angle: drag on hub and panel [rad]')
    plt.grid()
    pltName = fileName + "5" + planetCase
    figureList[pltName] = plt.figure(5)

    plt.figure(6)
    fig = plt.gcf()
    ax = fig.gca()
    for idx in range(3):
        plt.plot(dataLog1.times()*macros.NANO2MIN, dataOmegaBN_B1[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3), linestyle='--', # dash not working??
                 label=r'$\omega_' + str(idx) + '$'+ 'Drag on hub only')
    for idx in range(3): # they overlap, is this intended????
        plt.plot(dataLog2.times()*macros.NANO2MIN, dataOmegaBN_B2[:, idx],
                 color=unitTestSupport.getLineColor(idx, 3), 
                 label=r'$\omega_' + str(idx) + '$'+ 'Drag on hub and panel')
    plt.legend(loc='lower right')   
    plt.xlabel('Time [min]')    
    plt.ylabel(r'Rate $\omega_{B/N}$')
    plt.grid()
    pltName = fileName + "6" + planetCase
    figureList[pltName] = plt.figure(6)

    # Plot diffence in altitudes
    plt.figure(7)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(dataLog1.times()*macros.NANO2MIN, ((r1-r_eq)/1e3) - ((r2-r_eq)/1e3))
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Diffrence in altitude r1-r2 [km]')
    pltName = fileName + "7" + planetCase
    figureList[pltName] = plt.figure(7)

    plt.figure(8)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(dataLog1.times()*macros.NANO2MIN, a1/1e3, label="Drag on hub only")
    plt.plot(dataLog2.times()*macros.NANO2MIN, a2/1e3, label="Drag on hub and panel")
    plt.legend(loc='lower right')   
    plt.xlabel('Time [min]')    
    plt.ylabel('Semimajor axis [km]')
    plt.grid()
    pltName = fileName + "8" + planetCase
    figureList[pltName] = plt.figure(8)

    plt.figure(9)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(dataLog1.times()*macros.NANO2MIN, period1, label="Drag on hub only")    
    plt.plot(dataLog2.times()*macros.NANO2MIN, period2, label="Drag on hub and panel") 
    plt.legend(loc='lower right')   
    plt.xlabel('Time [min]')    
    plt.ylabel('Period [s]') # check units !!!!
    plt.grid()
    pltName = fileName + "9" + planetCase
    figureList[pltName] = plt.figure(9)

    plt.figure(10)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    plt.plot(dataNewAtmoLog.times()*macros.NANO2MIN, densData)
    plt.xlabel('Time [min]')
    plt.ylabel('Density in kg/m^3')
    pltName = fileName + "10" + planetCase
    figureList[pltName] = plt.figure(10)

    plt.figure(11)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='sci')
    plt.plot(densData, (r1-r_eq)/1e3, label="Drag on hub only")
    plt.plot(densData, (r2-r_eq)/1e3, label="Drag on hub and panel")
    plt.xlabel('Density in kg/m^3')
    plt.ylabel('Altitude [km]')
    pltName = fileName + "11" + planetCase
    figureList[pltName] = plt.figure(11)

    # rotating panel hinge angle: panel1 and panel3
    plt.figure(12)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(p1Log.times()*macros.NANO2MIN, panel1thetaLog, color='#aa0000', label="Drag on hub only")    
    plt.plot(p3Log.times()*macros.NANO2MIN, panel3thetaLog, color='#aa0000') 
    plt.plot(p1Log.times()*macros.NANO2MIN, panel2thetaLog, color='#008800', label="Drag on hub and panels")    
    plt.plot(p3Log.times()*macros.NANO2MIN, panel4thetaLog, color='#008800') 
    plt.legend(loc='lower right')   
    plt.xlabel('Time [min]')    
    plt.ylabel('Panel Angle [rad]')
    plt.grid()
    pltName = fileName + "4" + planetCase
    figureList[pltName] = plt.figure(12)

    if DOF == "2":
        # 2DOF angles grouped by spacecraft: SC1 (Panels 1 and 3, hinges 1 and 2)
        plt.figure(13)
        plt.plot(p1Log.times()*macros.NANO2MIN, panel1thetaLog, label="Panel1 H1")
        plt.plot(p2Log.times()*macros.NANO2MIN, panel2thetaLog, linestyle='--', label="Panel1 H2")
        plt.plot(p5Log.times()*macros.NANO2MIN, panel5thetaLog, label="Panel3 H1")
        plt.plot(p6Log.times()*macros.NANO2MIN, panel6thetaLog, linestyle='--', label="Panel3 H2")
        plt.legend(loc='lower right')
        plt.xlabel('Time [min]')
        plt.ylabel('SC1 Panel Angles [rad]')
        plt.grid()
        pltName = fileName + "13" + planetCase
        figureList[pltName] = plt.figure(13)

        # 2DOF angles grouped by spacecraft: SC2 (Panels 2 and 4, hinges 1 and 2)
        plt.figure(14)
        plt.plot(p3Log.times()*macros.NANO2MIN, panel3thetaLog, label="Panel2 H1")
        plt.plot(p4Log.times()*macros.NANO2MIN, panel4thetaLog, linestyle='--', label="Panel2 H2")
        plt.plot(p7Log.times()*macros.NANO2MIN, panel7thetaLog, label="Panel4 H1")
        plt.plot(p8Log.times()*macros.NANO2MIN, panel8thetaLog, linestyle='--', label="Panel4 H2")
        plt.legend(loc='lower right')
        plt.xlabel('Time [min]')
        plt.ylabel('SC2 Panel Angles [rad]')
        plt.grid()
        pltName = fileName + "14" + planetCase
        figureList[pltName] = plt.figure(14)

        # 2DOF hinge rates for all panels
        plt.figure(15)
        plt.plot(p1Log.times()*macros.NANO2MIN, panel1thetaDotLog, label="SC1 Panel1 H1Dot")
        plt.plot(p3Log.times()*macros.NANO2MIN, panel3thetaDotLog, label="SC2 Panel2 H1Dot")
        plt.plot(p5Log.times()*macros.NANO2MIN, panel5thetaDotLog, label="SC1 Panel3 H1Dot")
        plt.plot(p7Log.times()*macros.NANO2MIN, panel7thetaDotLog, label="SC2 Panel4 H1Dot")
        plt.plot(p2Log.times()*macros.NANO2MIN, panel2thetaDotLog, linestyle='--', label="SC1 Panel1 H2Dot")
        plt.plot(p4Log.times()*macros.NANO2MIN, panel4thetaDotLog, linestyle='--', label="SC2 Panel2 H2Dot")
        plt.plot(p6Log.times()*macros.NANO2MIN, panel6thetaDotLog, linestyle='--', label="SC1 Panel3 H2Dot")
        plt.plot(p8Log.times()*macros.NANO2MIN, panel8thetaDotLog, linestyle='--', label="SC2 Panel4 H2Dot")
        plt.legend(loc='lower right')
        plt.xlabel('Time [min]')
        plt.ylabel('Hinge Rate [rad/s]')
        plt.grid()
        pltName = fileName + "15" + planetCase
        figureList[pltName] = plt.figure(15)

    plt.show()
    plt.close("all")

    return figureList

# close the plots being saved off to avoid over-writing old and new figures
if __name__ == '__main__':
    run('Earth', '2', deorbitAlt=80)      # planet arrival case, can be Earth or Mars
