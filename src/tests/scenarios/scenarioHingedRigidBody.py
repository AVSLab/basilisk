''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

 Permission to use, copy, modify, and/or distribute this software for any
 purpose with or without fee is hereby granted, provided that the above
 copyright notice and this permission notice appear in all copies.

 THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

'''


#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Integrated tutorial of the spacecraftPlus(), gravity, and hinged rigid body modules illustrating
#           how Delta_v maneuver from scenarioOrbitManeuver.py affects the motion of the hinged rigid bodies.
#           Rotational motion is allowed on the spacecraft to simulate the full interaction of the hinged rigid
#           bodies and the spacecraft.
# Author:   Scott Carnahan
# Creation Date:  Jul. 17, 2017
#


import os
import numpy as np
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass  # The class which contains the basilisk simuation environment
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.utilities import macros  # Some unit conversions
from Basilisk.utilities import orbitalMotion
# import simulation related support
from Basilisk.simulation import \
    spacecraftPlus  # The base of any spacecraft simulation which deals with spacecraft dynamics
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import hingedRigidBodyStateEffector
# Allows for forces to act on the spacecraft without adding an effector like a thruster
from Basilisk.simulation import extForceTorque
# import non-basilisk libraries
import matplotlib.pyplot as plt



# NOTE: The unit test in this tutorial essentially only checks if the results are a very specific value. It will not
# work with other input conditions. It serves only to make sure that this specific tutorial is working when pytest is
# run and does not prove the functionality of any of the modules used.


## \defgroup Tutorials_3_1
## @{
## Demonstrates setting up hinged panels on a rigid spacecraft hub
#
# A Spacecraft with Hinged Rigid Bodies {#scenarioHingedRigidBody}
# ====
#
# Scenario Description
# -----
# This script sets up a 8-DOF spacecraft (3 translational, 3 rotational and 2 solar panel DOFs) which is orbiting Earth. It is nearly identical to the spacecraft
# which is demonstrated in [scenarioOrbitManeuver.py](@ref scenarioOrbitManeuver).  The purpose
# is to illustrate the use of the hinged rigid body module and illustrate the effects that a disturbance has on
# hinged rigid body motion.  Read [scenarioOrbitManeuver.py](@ref scenarioOrbitManeuver) to learn how to setup a
# basic spacecraft with impulsive Delta-v maneuvers. The scenario in this tutorial is similar to the  Hohmann
#  maneuver in the other scenario except that the length of the simulation is shorter and a non-impulsive Delta-v is applied through
# the external force and torque module. The shortened length of the simulation execution means that the maneuvers
# don't happen at the same point, so the effects of the maneuver are different than before.
# This scenario does not have multiple maneuver types, so nothing needs to be changed to run the scenario as was necessary
# in the orbit maneuvers tutorial
#
# To run the default scenario, call the python script through
#
#       python scenarioHingedRigidBody.py
#
# The simulation layout is shown in the following illustration.
# ![Simulation Flow Diagram](Images/doc/test_scenarioHingedRigidBody.svg "Illustration")
# A single simulation process is created
# which contains the spacecraft object and two hinged rigid bodies (panel1 and panel2). It should be noted here that "hinged rigid bodies"
# are rigid bodies which are hinged to the spacecraft hub by a single axis and they can rotate only about
# that axis and cannot translate. Details and graphics of the hinged rigid body can be found in the hinged rigid body
# documentation. Additionally, the spacecraft is orbiting earth, so a simIncludeGravBody is created and called
# earth. Finally, and external force is created and added to the spacecraft called extFTObject.
#
# The BSK simulation is run for a fixed period.  After stopping, the
# ExtForceTorque() module is given a non-zero external force value.
# When the simulation completes 4 plots are shown.  One plot always shows
# the inertial position vector components, while the second shows a plot
# of the orbital radius time history. In addition, there is a plot for the angular
# displacement of each hinged rigid body. The plots are different because the hinged rigid bodies were attached to
# the spacecraft hub at logical starting positions, but the thrust is applied to the hub in a constant inertial
# direction which is insignificant to the hinged rigid bodies. Therefore, the force has asymmetrical effects on the
# hinged rigid bodies.
#
# Rather than focusing only on how this simulation works, it may be more instructive to focus on the differences
# necessary to make this simulation work when adding the hinged rigid bodies to the spacecraft as well as the external
# force.
#
# The first change necessary is in the import statements at the beginning of the test scenario. Now,
# hingedRigidBodyStateEffector() and ExtForceTorque() must be imported.
# ~~~~~~~~~~~~~~~~~{.py}
# import hingedRigidBodyStateEffector
# import ExtForceTorque
# ~~~~~~~~~~~~~~~~~
#
# Next, the simulation time step should be reduced. Previously, the time step was easily set to 10 seconds because
# only orbital dynamics were being modelled. As will be seen in the plots from this tutorial, though, the panels will
# "flap" at relatively high frequency. Large time steps would not allow for this motion to be solved for correctly. In
# fact, with the 10 second time step, the simulation will not even run. This is a good reminder to check the time step
# size when trouble-shooting Basilisk simulations.
# ~~~~~~~~~~~~~~~~~{.py}
# simulationTimeStep = macros.sec2nano(0.1)
# ~~~~~~~~~~~~~~~~~
#
# Importantly, the hinged rigid body and external force are added next. This entire section is clearly marked in the
# code, but is repeated here for clarity:
# ~~~~~~~~~~~~~~~~~{.py}
# scSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
# scSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
#
# # Define Variable for panel 1
# scSim.panel1.mass = 100.0
# scSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
# scSim.panel1.d = 1.5
# scSim.panel1.k = 1000.0
# scSim.panel1.c = 0.0 #c is the rotational damping coefficient for the hinge, which is modeled as a spring.
# scSim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
# scSim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
# scSim.panel1.nameOfThetaState = "hingedRigidBodyTheta1"
# scSim.panel1.nameOfThetaDotState = "hingedRigidBodyThetaDot1"
# scSim.panel1.thetaInit = 5 * np.pi / 180.0
# scSim.panel1.thetaDotInit = 0.0
# scSim.panel1.HingedRigidBodyOutMsgName = "panel1Msg"
#
# # Define Variables for panel 2
# scSim.panel2.mass = 100.0
# scSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
# scSim.panel2.d = 1.5
# scSim.panel2.k = 1000.
# scSim.panel2.c = 0.0 #c is the rotational damping coefficient for the hinge, which is modeled as a spring.
# scSim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
# scSim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
# scSim.panel2.nameOfThetaState = "hingedRigidBodyTheta2"
# scSim.panel2.nameOfThetaDotState = "hingedRigidBodyThetaDot2"
# scSim.panel2.thetaInit = 5 * np.pi / 180.0
# scSim.panel2.thetaDotInit = 0.0
# scSim.panel2.HingedRigidBodyOutMsgName = "panel2Msg"
#
# # Add panels to spaceCraft
# scObject.addStateEffector(scSim.panel1)  # in order to affect dynamics
# scObject.addStateEffector(scSim.panel2)  # in order to affect dynamics
#
# scSim.AddModelToTask(simTaskName, scSim.panel1)  # in order to track messages
# scSim.AddModelToTask(simTaskName, scSim.panel2)  # in order to track messages
#
# # Define mass properties of the rigid part of the spacecraft
# scObject.hub.mHub = 800.0
# scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
# scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
#
# scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
# scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]
#
# # setup extForceTorque module
# extFTObject = extForceTorque.ExtForceTorque()
# extFTObject.ModelTag = "maneuverThrust"
# extFTObject.extForce_N = [[0.], [0.], [0.]]
# scObject.addDynamicEffector(extFTObject)
# scSim.AddModelToTask(simTaskName, extFTObject)
# ~~~~~~~~~~~~~~~~~
# Above, it is important to understand that just creating the hinged rigid body is not enough to have it affect the
# simulation. In order for it to affect spacecraft dynamics, it must also be added to the spacecraft:
# ~~~~~~~~~~~~~~~~~{.py}
# scObject.addStateEffector(scSim.panel1)  # in order to affect dynamics
# scObject.addStateEffector(scSim.panel2)  # in order to affect dynamics
# ~~~~~~~~~~~~~~~~~
# Additionally, plotting the panel states (theta) at the end of the simulation requires use of the messaging system.
# In order for the messages to be logged, the panels must be added to the simulation dynamics task:
# ~~~~~~~~~~~~~~~~~{.py}
# scSim.AddModelToTask(simTaskName, scSim.panel1)  # in order to track messages
# scSim.AddModelToTask(simTaskName, scSim.panel2)  # in order to track messages
# ~~~~~~~~~~~~~~~~~
# The same goes for the external force:
# ~~~~~~~~~~~~~~~~~{.py}
# scObject.addDynamicEffector(extFTObject)
# scSim.AddModelToTask(simTaskName, extFTObject)
# ~~~~~~~~~~~~~~~~~
# There are additional lines that must be added in order to retrieve and plot the data from the panel messages. These
# lines are split before and after the simulation executes:
# ~~~~~~~~~~~~~~~~~{.py}
# scSim.TotalSim.logThisMessage(scSim.panel1.HingedRigidBodyOutMsgName, samplingTime)
# scSim.TotalSim.logThisMessage(scSim.panel2.HingedRigidBodyOutMsgName, samplingTime)
# ...
# ...
# ...
# panel1thetaLog = scSim.pullMessageLogData(scSim.panel1.HingedRigidBodyOutMsgName+'.theta',range(1))
# panel2thetaLog = scSim.pullMessageLogData(scSim.panel2.HingedRigidBodyOutMsgName+'.theta',range(1))
# ~~~~~~~~~~~~~~~~~
# The panel theta logs are queried with "range(1)" rather than "range(3)" because theta is a scalar rather than a 3-D
# vector like the position and velocities which are retrieved in the lines just before the panel theta logs. The
# Hinged Rigid Body module is also set up with a message for "thetaDot" which can be retrieved by replacing ".theta"
# with ".thetaDot".
# Moving on, the orbit maneuver code must be changed to  implement the finite thrusting maneuver rather than the
# impulse Delta-v used before. The code which is removed is not shown here, but can be seen by comparing the orbit
# change maneuver tutorial to this one. When complete, this section of the code now looks like:
# ~~~~~~~~~~~~~~~~~{.py}
# Hohmann transfer
# extFTObject.extForce_N = [[-2050.], [-1430.], [-.00076]]
# T2 = macros.sec2nano(935.)  # this is the amount of time to get a deltaV equal to what the other tutorial has
# ~~~~~~~~~~~~~~~~~
# Finally, the second and third orbit maneuvers have been removed from this tutorial. The intended demonstration is already complete,
# and the smaller time steps necessary here make it wasteful to simulate more than is necessary. Aside from these
# changes, other variables used in instantaneous Delta-V calculations have been removed.
#
# If a user has time, it would be a good exercise to attempt to model the same orbits and maneuvers as
# scenarioOrbitManeuver.py but with non-impulsive Delta-v and to examine the differences in the after-
# maneuver orbits. Be aware that those simulations will require a long time (tens of minutes) to run.
#
# Setup
# -----
#
# The main call for this script is the run() command.  Just calling the script with python invokes this call:
# ~~~~~~~~~~~~~{.py}
# if __name__ == "__main__":
#     run(
#          True,        # show_plots
#        )
# ~~~~~~~~~~~~~
# In this scenario something similar to a classical Hohmann transfer is being
# simulated to go from LEO to reach and stay at GEO, but with a finite thrusting time. The math behind such maneuvers
# can be found in textbooks such as *Analytical Mechanics of Space Systems*
# (<http://arc.aiaa.org/doi/book/10.2514/4.102400>).
# The resulting position coordinates and orbit illustration are shown below.
# ![Inertial Position Coordinates History](Images/Scenarios/scenarioHingedRigidBody10.svg "Position history")
# ![Orbit Radius Illustration](Images/Scenarios/scenarioHingedRigidBody20.svg "Radius Illustration")
#
# The hinged rigid bodies were given an initial angular displacement. Then, the externally applied force caused
# greater displacement. As discussed above, the reaction is asymmetric between the panels due to panel orientation.
# Another interesting result is that, during the thrusting maneuver, the hinged bodies oscillate about a non-zero point.
# This is because they are under a constant, non-zero acceleration, similar to a weight hanging from a spring on Earth.
# The springs know no difference between a gravity field and acceleration.
# ![Panel 1 Displacement](Images/Scenarios/scenarioHingedRigidBodypanel1theta0.svg "Panel 1 Theta Illustration")
# ![Panel 2 Displacement](Images/Scenarios/scenarioHingedRigidBodypanel2theta0.svg "Panel 2 Theta Illustration")
#
#
## @}

def run(show_plots):
    '''Call this routine directly to run the tutorial scenario.'''


    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()
    scSim.TotalSim.terminateSimulation()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # unitTestSupport.enableVisualization(scSim, dynProcess)

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    mu = earth.mu

    # Attach gravity model to spacecraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    # Adding the HingedRigidBody State Effector
    scSim.panel1 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()
    scSim.panel2 = hingedRigidBodyStateEffector.HingedRigidBodyStateEffector()

    # Define Variable for panel 1
    scSim.panel1.mass = 100.0
    scSim.panel1.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panel1.d = 1.5
    scSim.panel1.k = 1000.0
    scSim.panel1.c = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    scSim.panel1.r_HB_B = [[0.5], [0.0], [1.0]]
    scSim.panel1.dcm_HB = [[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]]
    scSim.panel1.nameOfThetaState = "hingedRigidBodyTheta1"
    scSim.panel1.nameOfThetaDotState = "hingedRigidBodyThetaDot1"
    scSim.panel1.thetaInit = 5 * np.pi / 180.0
    scSim.panel1.thetaDotInit = 0.0
    scSim.panel1.HingedRigidBodyOutMsgName = "panel1Msg"

    # Define Variables for panel 2
    scSim.panel2.mass = 100.0
    scSim.panel2.IPntS_S = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panel2.d = 1.5
    scSim.panel2.k = 1000.
    scSim.panel2.c = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    scSim.panel2.r_HB_B = [[-0.5], [0.0], [1.0]]
    scSim.panel2.dcm_HB = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    scSim.panel2.nameOfThetaState = "hingedRigidBodyTheta2"
    scSim.panel2.nameOfThetaDotState = "hingedRigidBodyThetaDot2"
    scSim.panel2.thetaInit = 5 * np.pi / 180.0
    scSim.panel2.thetaDotInit = 0.0
    scSim.panel2.HingedRigidBodyOutMsgName = "panel2Msg"

    # Add panels to spaceCraft
    scObject.addStateEffector(scSim.panel1)  # in order to affect dynamics
    scObject.addStateEffector(scSim.panel2)  # in order to affect dynamics

    scSim.AddModelToTask(simTaskName, scSim.panel1)  # in order to track messages
    scSim.AddModelToTask(simTaskName, scSim.panel2)  # in order to track messages

    # Define mass properties of the rigid part of the spacecraft
    scObject.hub.mHub = 800.0
    scObject.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]

    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]

    # setup extForceTorque module
    extFTObject = extForceTorque.ExtForceTorque()
    extFTObject.ModelTag = "maneuverThrust"
    extFTObject.extForce_N = [[0.], [0.], [0.]]
    scObject.addDynamicEffector(extFTObject)
    scSim.AddModelToTask(simTaskName, extFTObject)
    # Ending the HingedRigidBody State Effector

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000  # meters
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m - v_CN_N

    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTimeFactor = 0.01
    simulationTime = macros.sec2nano(simulationTimeFactor * P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints - 1)
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(scSim.panel1.HingedRigidBodyOutMsgName, samplingTime)
    scSim.TotalSim.logThisMessage(scSim.panel2.HingedRigidBodyOutMsgName, samplingTime)

    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    # compute maneuver Delta_v's
    extFTObject.extForce_N = [[-2050.], [-1430.], [-.00076]]
    T2 = macros.sec2nano(935.)  # this is the amount of time to get a deltaV equal to what the other tutorial has.

    # run simulation for 2nd chunk
    scSim.ConfigureStopTime(simulationTime + T2)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.r_BN_N', range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName + '.v_BN_N', range(3))
    panel1thetaLog = scSim.pullMessageLogData(scSim.panel1.HingedRigidBodyOutMsgName + '.theta', range(1))
    panel2thetaLog = scSim.pullMessageLogData(scSim.panel2.HingedRigidBodyOutMsgName + '.theta', range(1))
    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileName = os.path.basename(os.path.splitext(__file__)[0])

    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    for idx in range(1, 4):
        plt.plot(posData[:, 0] * macros.NANO2HOUR, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [h]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1" + str(int(0.))
    figureList[pltName] = plt.figure(1)

    # show SMA
    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    rData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem_parab(mu, posData[idx, 1:4], velData[idx, 1:4])
        rData.append(oeData.rmag / 1000.)
    plt.plot(posData[:, 0] * macros.NANO2MIN, rData, color='#aa0000',
             )
    plt.xlabel('Time [min]')
    plt.ylabel('Radius [km]')
    pltName = fileName + "2" + str(int(0.))
    figureList[pltName] = plt.figure(2)

    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    plt.plot(panel1thetaLog[:, 0] * macros.NANO2MIN, panel1thetaLog[:, 1])
    plt.xlabel('Time [min]')
    plt.ylabel('Panel 1 Angular Displacement [r]')
    pltName = fileName + "panel1theta" + str(int(0.))
    figureList[pltName] = plt.figure(3)

    plt.figure(4)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    plt.plot(panel2thetaLog[:, 0] * macros.NANO2MIN, panel2thetaLog[:, 1])
    plt.xlabel('Time [min]')
    plt.ylabel('Panel 2 Angular Displacement [r]')
    pltName = fileName + "panel2theta" + str(int(0.))
    figureList[pltName] = plt.figure(4)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return velData, figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True  # show_plots
    )
