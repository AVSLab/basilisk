
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


def run(show_plots, planetCase, deorbitAlt=90):
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
    simulationTimeStep = macros.sec2nano(0.1)
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
    drag1.ModelTag = "FacetDrag"
    dragEffectorTaskName1 = "drag" #should there be one task name per drag1 and drag 2?
    # drag1.setDensityMessage(atmoModule.envOutMsgs[0]) # docs say to do this but scenarioDragRendezvous does not do it

    scAreas = [10.0, 10.0]
    scCoeff = np.array([2.0, 2.0])
    
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
    
    for ind in range(0,len(scAreas)):
        drag1.addFacet(scAreas[ind], scCoeff[ind], B_normals[ind], B_locations[ind])

    # Drag Effectors for sc2

    drag2 = facetDragDynamicEffector.FacetDragDynamicEffector()
    drag2.ModelTag = "FacetDrag"
    dragEffectorTaskName2 = "drag"
    # drag2.setDensityMessage(atmoModule.envOutMsgs[0]) # docs say to do this but scenarioDragRendezvous does not do it

    # sc2 has the same hub facets as sc1:
    for ind in range(0,len(scAreas)):
        drag2.addFacet(scAreas[ind], scCoeff[ind], B_normals[ind], B_locations[ind])

    drag3 = facetDragDynamicEffector.FacetDragDynamicEffector()
    drag3.ModelTag = "FacetDrag"
    dragEffectorTaskName3 = "panelDrag"
    
    panelArea = 25    # m^2 ???
    panelOffset = 0.3  # Distance from hub COM
    panelCd = 5

    panel_normals = [
        np.array([0, 0,  1]), 
        np.array([0, 0, -1])
    ]

    panel_locations = [
        np.array([0.0, 0.0, panelOffset]),
        np.array([0.0, 0.0, panelOffset])
    ]

    for i in range(2):
        drag3.addFacet(panelArea, panelCd, panel_normals[i], panel_locations[i])

    
    for i in range(2):
        drag1.addFacet(panelArea, panelCd, panel_normals[i], panel_locations[i])

    # Add another drag and panel facets for sc2 later there:

    


    dynProcess.addTask(scSim.CreateNewTask(atmoTaskName, simulationTimeStep))
    
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName1, simulationTimeStep))
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName2, simulationTimeStep))
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName3, simulationTimeStep))
    
    scSim.AddModelToTask(atmoTaskName, tabAtmo)

    # Add test module to runtime call list
    scSim.AddModelToTask(simTaskName, tabAtmo)

    #
    #   setup the simulation tasks/objects
    #

    m_sc = 2530.0    # kg

    # initialize spacecraft object and set properties
    scObject1 = spacecraft.Spacecraft() # panel fixed over time from POV of drag 
    scObject1.ModelTag = "spacecraftBody"
    scObject1.hub.mHub = m_sc
    tabAtmo.addSpacecraftToModel(scObject1.scStateOutMsg)

    scObject2 = spacecraft.Spacecraft() # one facited drag to hub and another attached to panel (2 - one for each side)
    scObject2.ModelTag = "spacecraftBody"
    scObject2.hub.mHub = m_sc
    tabAtmo.addSpacecraftToModel(scObject2.scStateOutMsg)
    
    simpleNavObj = simpleNav.SimpleNav()
    scSim.AddModelToTask(simTaskName, simpleNavObj)
    simpleNavObj.scStateInMsg.subscribeTo(scObject1.scStateOutMsg)
    simpleNavObj.scStateInMsg.subscribeTo(scObject2.scStateOutMsg)

    # Panel 1 is not the parent to the drag effector 
    scSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector() 
    scSim.panel1.ModelTag = "panel1"

    # Panel 2 is the parent to the drag effector (and so is the hub)
    scSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    scSim.panel2.ModelTag = "panel2"

    scSim.panel1.mass = 200
    scSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panel1.d = 16
    scSim.panel1.k = 20
    scSim.panel1.c = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    scSim.panel1.r_HB_B = [[-0.5], [0.0], [1.0]]
    scSim.panel1.dcm_HB = [[1, 0, 0.0], [0.0, 1, 0.0], [0.0, 0.0, 1]]
    scSim.panel1.thetaInit = 0.0
    scSim.panel1.thetaDotInit = 0.0

    scSim.panel2.mass = 200
    scSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panel2.d = 16
    scSim.panel2.k = 20
    scSim.panel2.c = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    scSim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]] 
    scSim.panel2.dcm_HB = [[1, 0, 0.0], [0.0, 1, 0.0], [0.0, 0.0, 1]]
    scSim.panel2.thetaInit = 0.0
    scSim.panel2.thetaDotInit = 0.0

    # # Symmetrically opposite panels for each spacecraft
    # scSim.panel3 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector() 
    # scSim.panel3.ModelTag = "panel3"

    # scSim.panel4 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector() 
    # scSim.panel4.ModelTag = "panel4"

    # scSim.panel3.mass = 200
    # scSim.panel3.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    # scSim.panel3.d = -16
    # scSim.panel3.k = 20
    # scSim.panel3.c = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    # scSim.panel3.r_HB_B = [[0.5], [0.0], [1.0]]
    # scSim.panel3.dcm_HB = [[1, 0, 0.0], [0.0, 1, 0.0], [0.0, 0.0, 1]]
    # scSim.panel3.thetaInit = 0.0
    # scSim.panel3.thetaDotInit = 0.0

    # scSim.panel4.mass = 200
    # scSim.panel4.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    # scSim.panel4.d = -16
    # scSim.panel4.k = 20
    # scSim.panel4.c = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    # scSim.panel4.r_HB_B = [[0.5], [0.0], [1.0]] 
    # scSim.panel4.dcm_HB = [[1, 0, 0.0], [0.0, 1, 0.0], [0.0, 0.0, 1]]
    # scSim.panel4.thetaInit = 0.0
    # scSim.panel4.thetaDotInit = 0.0

    #########
    # Add panels to spaceCraft
    #########

    # in order to affect dynamics
    scObject1.addStateEffector(scSim.panel1)
    scObject2.addStateEffector(scSim.panel2)
    
    # scObject1.addStateEffector(scSim.panel3)
    # scObject2.addStateEffector(scSim.panel4)

    # in order to track messages
    scSim.AddModelToTask(simTaskName, scSim.panel1) 
    scSim.AddModelToTask(simTaskName, scSim.panel2)
    # scSim.AddModelToTask(simTaskName, scSim.panel3)
    # scSim.AddModelToTask(simTaskName, scSim.panel4)
    
    # Attach drag to hub only (previous method)
    scObject1.addDynamicEffector(drag1) # remember later to add the other panel facets to the drag!!!
    
    # Attach drag to hub and panel (new/more accurate model)
    scObject2.addDynamicEffector(drag2)
    # scSim.panel2.addDynamicEffector(drag3) 
    # scSim.panel4.addDynamicEffector(drag4) # REMEMBER TO ADD A 4th drag for the second panel 

    # Add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject1)
    scSim.AddModelToTask(simTaskName, scObject2)

    scSim.AddModelToTask(dragEffectorTaskName1, drag1)
    scSim.AddModelToTask(dragEffectorTaskName2, drag2)
    scSim.AddModelToTask(dragEffectorTaskName2, drag3)
    # scSim.AddModelToTask(dragEffectorTaskName2, drag4)


    # Clear prior gravitational body and SPICE setup definitions

    drag1.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])
    drag2.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])
    drag3.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])
    # drag4.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])

    # Setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    planet = gravFactory.createBody(planetCase)
    planet.isCentralBody = True  # ensure this is the central gravitational body
    mu = planet.mu

    # Attach gravity model to spacecraft
    gravFactory.addBodiesTo(scObject1)
    gravFactory.addBodiesTo(scObject2)

    if planetCase == 'Earth':
        r = 6503 * 1000
        u = 11.2 * 1000
        gam = -5.15 * macros.D2R
    else:
        r = (3397.2 + 125.) * 1000
        u = 6 * 1000
        gam = -10 * macros.D2R
    lon = 0
    lat = 0
    hda = np.pi/2
    xxsph = [r,lon,lat,u,gam,hda]
    rN, vN = sph2rv(xxsph)
    
    scObject1.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject1.hub.v_CN_NInit = vN  # m - v_CN_N

    scObject2.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject2.hub.v_CN_NInit = vN  # m - v_CN_N

    # set the simulation time
    if planetCase == 'Earth':
        simulationTime = macros.sec2nano(300)
    else:
        simulationTime = macros.sec2nano(400)

    #
    #   Setup data logging before the simulation is initialized
    #

    dataLog1 = scObject1.scStateOutMsg.recorder()
    dataLog2 = scObject2.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, dataLog1)
    scSim.AddModelToTask(simTaskName, dataLog2)

    dataNewAtmoLog = tabAtmo.envOutMsgs[0].recorder()
    scSim.AddModelToTask(simTaskName, dataNewAtmoLog)

    #
    #   initialize Spacecraft States with initialization variables
    #
    scObject1.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject1.hub.v_CN_NInit = vN  # m - v_CN_N

    scObject2.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject2.hub.v_CN_NInit = vN  # m - v_CN_N

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

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(scSim, simTaskName, [
                                        scObject1, 
                                        scObject2
                                        ],
                                        saveFile=fileName
                                        )
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
    posData2 = dataLog1.r_BN_N
    velData2 = dataLog1.v_BN_N

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
        plt.plot(dataLog1.times()*macros.NANO2MIN, posData1[:, idx]/1000.,
                 color=unitTestSupport.getLineColor(idx,3),
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
    plt.plot(v2/1e3, (r2-r_eq)/1e3, label="Drag on hub and panel")
    plt.xlabel('velocity [km/s]')
    plt.ylabel('altitude [km]')
    plt.grid()
    pltName = fileName + "4" + planetCase
    figureList[pltName] = plt.figure(4)

    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(dataLog1.times()*macros.NANO2MIN, (r1-r_eq)/1e3, label="Drag on hub only")
    plt.plot(dataLog2.times()*macros.NANO2MIN, (r2-r_eq)/1e3, label="Drag on hub and panel")
    plt.xlabel('time [min]')
    plt.ylabel('altitude [km]')
    plt.grid()
    pltName = fileName + "5" + planetCase
    figureList[pltName] = plt.figure(3)

    if show_plots:
        plt.show()
        plt.close("all")

    return figureList

    # close the plots being saved off to avoid over-writing old and new figures
if __name__ == '__main__':
    run(True, 'Earth', deorbitAlt=90)      # planet arrival case, can be Earth or Mars
    