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
# Purpose:  Demonstration of how to setup and use different integrators in
#           Basilisk.  The simulation performs a 3-DOF orbit scenario.
# Author:   Hanspeter Schaub
# Creation Date:  Dec. 14, 2016
#

import os
import numpy as np

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion
# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import svIntegrators




## \defgroup Tutorials_1_1
##   @{
## How to setup different integration methods for a basic 3-DOF orbit scenario.
#
# Specifying a Dynamics Integrator Module {#scenarioIntegrators}
# ====
#
# Scenario Description
# -----
# This script sets up a 3-DOF spacecraft which is orbiting a Earth.  The purpose
# is to illustrate how to specify a particular integrator to be used.
# The scenarios can be run with the followings setups parameters:
# Setup | integratorCase
# ----- | -------------------
# 1     | "rk4" (RK4 - default)
# 2     | "euler" (Euler)
# 3     | "rk2" (RK2)
#
# To run the default scenario 1., call the python script through
#
#       python scenarioIntegrators.py
#
# When the simulation completes a plot is shown for illustrating both the true and the numerically
# evaluated orbit.
#
# The simulation layout is shown in the following illustration.  A single simulation process is created
# which contains the spacecraft object.  Gravity effectors are attached to the spacecraft dynamics to
# simulate the gravitational accelerations.  The spacecraft object provides the states that the integration
# module needs to perform the time integration.
# ![Simulation Flow Diagram](Images/doc/test_scenarioIntegrators.svg "Illustration")
#
# If spacecraftPlus(), or any other dynamics module, is created without specifying a particular
# integration type, the fixed time step 4th order Runge-Kutta method is used by default.  To invoke a
# different integration scheme, the following code is used before the dynamics module is added to the
# python task list:
#~~~~~~~~~~~~~~~~~{.py}
#   integratorObject = svIntegrators.svIntegratorEuler(scObject)
#   scObject.setIntegrator(integratorObject)
#~~~~~~~~~~~~~~~~~
# The first line invokes an instance of the desired state vector integration module, and provides
# the dynamics module (spacecraftPlus() in this case) as the input.  This specifies to the integrator
# module which other module will provide the equationOfMotion() function to evaluate the derivatives of
# the state vector.  The send line ties the integration module to the dynamics module.  After that we are
# done.
#
# The integrator scenario script is setup to evaluate the default integration method (RK4), a first order
# Euler integration method, as well as a second order RK2 method.  The following figure illustrates the resulting
# trajectories relative to the true trajectory using a very coarse integration time step of 120 seconds.
# ![Orbit Illustration](Images/Scenarios/scenarioIntegrators.svg "orbit comparison")
# The RK4 method still approximates the true orbit well, while the RK2 method is starting to show some visible
# errors. The first order Euler method provides a horrible estimate of the resulting trajectory, illustrating
# that much smaller time steps must be used with this method in this scenario.
#
# Creating new Integration modules
# -----
#
# New integration modules can readily be created for Basilisk.  They are all stored in the folder
#~~~~~~~~~~~~~~~~~
#   Basilisk/simulation/dynamics/Integrators/
#~~~~~~~~~~~~~~~~~
# The integrators must be created to function on a general state vector and be independent of the particular
# dynamics being integrated.  Note that the default integrator is placed inside the `_GeneralModulesFiles`
# folder within the `dynamics` folder.
#
##  @}
def run(show_plots, integratorCase):
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
    simulationTimeStep = macros.sec2nano(120.)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # unitTestSupport.enableVisualization(scSim, dynProcess, simProcessName, 'earth')  # The Viz only support 'earth', 'mars', or 'sun'

    #
    #   setup the simulation tasks/objects
    #
    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    # default case, RK4 is automatically setup, no extra code is needed
    if integratorCase == "euler":
        integratorObject = svIntegrators.svIntegratorEuler(scObject)
        scObject.setIntegrator(integratorObject)
    elif integratorCase == "rk2":
        integratorObject = svIntegrators.svIntegratorRK2(scObject)
        scObject.setIntegrator(integratorObject)

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    # clear prior gravitational body and SPICE setup definitions
    gravFactory = simIncludeGravBody.gravBodyFactory()

    earth = gravFactory.createEarth()
    earth.isCentralBody = True  # ensure this is the central gravitational body
    mu = earth.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector(gravFactory.gravBodies.values())

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000.*1000  # meters
    oe.a = rLEO
    oe.e = 0.0001
    oe.i = 33.3*macros.D2R
    oe.Omega = 48.2*macros.D2R
    oe.omega = 347.8*macros.D2R
    oe.f = 85.3*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)
    #
    #   initialize Spacecraft States with in the initialization variables
    #
    scObject.hub.r_CN_NInit = unitTestSupport.np2EigenVectorXd(rN)  # m - r_CN_N
    scObject.hub.v_CN_NInit = unitTestSupport.np2EigenVectorXd(vN)  # m - v_CN_N

    # set the simulation time
    n = np.sqrt(mu/oe.a/oe.a/oe.a)
    P = 2.*np.pi/n
    simulationTime = macros.sec2nano(0.75*P)

    #
    #   Setup data logging before the simulation is initialized
    #
    numDataPoints = 100
    samplingTime = simulationTime / numDataPoints
    scSim.TotalSim.logThisMessage(scObject.scStateOutMsgName, samplingTime)


    #
    #   initialize Simulation
    #
    scSim.InitializeSimulationAndDiscover()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N', range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_BN_N', range(3))

    #
    #   plot the results
    #
    np.set_printoptions(precision=16)
    fileName = os.path.basename(os.path.splitext(__file__)[0])
    if integratorCase == "rk4":
        plt.close("all")        # clears out plots from earlier test runs

    # draw orbit in perifocal frame
    b = oe.a*np.sqrt(1-oe.e*oe.e)
    p = oe.a*(1-oe.e*oe.e)
    plt.figure(1,figsize=np.array((1.0, b/oe.a))*4.75,dpi=100)
    plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b])/1000*1.25)
    # draw the planet
    fig = plt.gcf()
    fig.set_tight_layout(False)
    ax = fig.gca()
    planetColor= '#008800'
    planetRadius = earth.radEquator/1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    labelStrings = ("rk4", "euler", "rk2")
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx, 1:4], velData[idx, 1:4])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
             , color=unitTestSupport.getLineColor(labelStrings.index(integratorCase)+1, 3)
             , label=integratorCase
             , linewidth=3.0
             )
    # draw the full osculating orbit from the initial conditions
    fData = np.linspace(0, 2*np.pi, 100)
    rData = []
    for idx in range(0, len(fData)):
        rData.append(p/(1+oe.e*np.cos(fData[idx])))
    plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
             , '--'
             , color='#555555'
             )
    plt.xlabel('$i_e$ Cord. [km]')
    plt.ylabel('$i_p$ Cord. [km]')
    plt.legend(loc='lower right')
    plt.grid()
    figureList = {}
    pltName = fileName
    figureList[pltName] = plt.figure(1)


    if show_plots:
        plt.show()

    # # close the plots being saved off to avoid over-writing old and new figures
    # plt.close("all")

    if integratorCase == "rk2":
        plt.close("all")

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return posData, figureList


#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run(
        True,        # show_plots
        'rk4')       # integrator case(0 - RK4, 1 - Euler, 2 - RK2)
