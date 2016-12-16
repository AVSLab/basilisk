''' '''
'''
 ISC License

 Copyright (c) 2016-2017, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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



import pytest
import sys, os, inspect
import matplotlib
import numpy as np
import ctypes
import math
import csv
import logging

# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0] + '/Basilisk/modules')
sys.path.append(splitPath[0] + '/Basilisk/PythonModules')
# @endcond

# import general simulation support files
import SimulationBaseClass
import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
import macros
import orbitalMotion

# import simulation related support
import spacecraftPlus
import simIncludeGravity

import svIntegrators






# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
@pytest.mark.xfail(True, reason="Scott's brain no-worky\n")

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("integratorCase", [
      (0)
    , (1)
    , (2)
])

# provide a unique test method name, starting with test_
def test_scenarioIntegrators(show_plots, integratorCase):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called
    [testResults, testMessage] = run( True,
            show_plots, integratorCase)
    assert testResults < 1, testMessage



## This scenario demonstrates how to setup basic 3-DOF orbits.
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
# 1     | 0 (RK4 - default)
# 2     | 1 (Euler)
# 3     | 2 (RK2)
#
# To run the default scenario 1., call the python script through
#
#       python test_scenarioIntegrators.py
#
# When the simulation completes a plot is shown for illustrating both the true and the numerically
# evaluated orbit.
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
# the state vector.  The send line ties the integration moduel to the dynamics module.  After that we are
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
# New integration modules can readily be created for Basilik.  They are all stored in the folder
#~~~~~~~~~~~~~~~~~
#   Basilisk/SimCode/dynamics/Integrators/
#~~~~~~~~~~~~~~~~~
# The integrators must be created to function on a general state vector and be independent of the particular
# dynamics being integrated.  Note that the default integrator is placed inside the `_GeneralModulesFiles`
# folder within the `dynamics` folder.

def run(doUnitTests, show_plots, integratorCase):
    '''Call this routine directly to run the tutorial scenario.'''
    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages

    #
    #  From here on there scenario python code is found.  Above this line the code is to setup a
    #  unitTest environment.  The above code is not critical if learning how to code BSK.
    #

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


    #
    #   setup the simulation tasks/objects
    #


    # initialize spacecraftPlus object and set properties
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = False

    # default case, RK4 is automatically setup, no extra code is needed
    if integratorCase == 1:
        integratorObject = svIntegrators.svIntegratorEuler(scObject)
        scObject.setIntegrator(integratorObject)
    elif integratorCase == 2:
        integratorObject = svIntegrators.svIntegratorRK2(scObject)
        scObject.setIntegrator(integratorObject)

    # add spacecraftPlus object to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)

    gravBody, ephemData = simIncludeGravity.addEarth()
    gravBody.isCentralBody = True          # ensure this is the central gravitational body
    mu = gravBody.mu

    # attach gravity model to spaceCraftPlus
    scObject.gravField.gravBodies = spacecraftPlus.GravBodyVector([gravBody])

    #
    #   setup orbit and simulation time
    #
    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000.*1000      # meters
    oe.a     = rLEO
    oe.e     = 0.0001
    oe.i     = 33.3*macros.D2R
    oe.Omega = 48.2*macros.D2R
    oe.omega = 347.8*macros.D2R
    oe.f     = 85.3*macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu,rN,vN)

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
    # create simulation messages
    #

    # create the gravity ephemerise message
    messageSize = ephemData.getStructSize()
    scSim.TotalSim.CreateNewMessage(simProcessName,
                                          gravBody.bodyInMsgName, messageSize, 2)
    scSim.TotalSim.WriteMessageData(gravBody.bodyInMsgName, messageSize, 0,
                                    ephemData)


    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()


    #
    #   initialize Spacecraft States within the state manager
    #   this must occur after the initialization
    #
    posRef = scObject.dynManager.getStateObject("hubPosition")
    velRef = scObject.dynManager.getStateObject("hubVelocity")

    posRef.setState(unitTestSupport.np2EigenVector3d(rN))  # m - r_BN_N
    velRef.setState(unitTestSupport.np2EigenVector3d(vN))  # m - v_BN_N


    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.r_BN_N',range(3))
    velData = scSim.pullMessageLogData(scObject.scStateOutMsgName+'.v_BN_N',range(3))

    np.set_printoptions(precision=16)

    #
    #   plot the results
    #
    fileNameString = filename[len(path)+6:-3]

    # draw orbit in perifocal frame
    b = oe.a*np.sqrt(1-oe.e*oe.e)
    p = oe.a*(1-oe.e*oe.e)
    plt.figure(1,figsize=np.array((1.0, b/oe.a))*4.75,dpi=100)
    plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b])/1000*1.25)
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor= '#008800'
    planetRadius = gravBody.radEquator/1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData=[]
    fData=[]
    labelString = ('RK4', 'Euler', 'RK2')
    for idx in range(0,len(posData)):
        oeData = orbitalMotion.rv2elem(mu,posData[idx,1:4],velData[idx,1:4])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
             ,color=unitTestSupport.getLineColor(integratorCase+1,3)
             ,label = labelString[integratorCase]
             ,linewidth = 3.0
             )
    # draw the full osculating orbit from the initial conditions
    fData = np.linspace(0,2*np.pi,100)
    rData = []
    for idx in range(0,len(fData)):
        rData.append(p/(1+oe.e*np.cos(fData[idx])))
    plt.plot(rData*np.cos(fData)/1000, rData*np.sin(fData)/1000
             ,'--'
             , color='#555555'
             )
    plt.xlabel('$i_e$ Cord. [km]')
    plt.ylabel('$i_p$ Cord. [km]')
    plt.legend(loc='lower right')
    plt.grid()
    if doUnitTests:     # only save off the figure if doing a unit test run
        unitTestSupport.saveScenarioFigure(
            fileNameString
            , plt, path)

    if show_plots:
        plt.show()

    # # close the plots being saved off to avoid over-writing old and new figures
    # plt.close("all")


    #
    #   the python code below is for the unit testing mode.  If you are studying the scenario
    #   to learn how to run BSK, you can stop reading below this line.
    #
    if doUnitTests:
        numTruthPoints = 5
        skipValue = int(len(posData)/(numTruthPoints-1))
        dataPosRed = posData[::skipValue]

        # setup truth data for unit test
        if integratorCase == 0 :
            truePos = [
                  [-2.8168016010234915e6,5.248174846916147e6,3.677157264677297e6]
                , [-6.379381726549218e6,-1.4688565370540658e6,2.4807857675497606e6]
                , [-2.230094305694789e6,-6.410420020364709e6,-1.7146277675541767e6]
                , [ 4.614900659014343e6,-3.60224207689023e6,-3.837022825958977e6]
                , [ 5.879095186201691e6,3.561495655367985e6,-1.3195821703218794e6]
            ]
        if integratorCase == 1 :
            truePos = [
                  [-2.8168016010234915e6,5.248174846916147e6,3.677157264677297e6]
                , [-7.061548530211288e6,-1.4488790844105487e6,2.823580168201031e6]
                , [-4.831279689590867e6,-8.015202650472983e6,-1.1434851461593418e6]
                , [ 719606.5825106134,-1.0537603309084207e7,-4.966060248346598e6]
                , [ 6.431097055190775e6,-9.795566286964862e6,-7.438012269629238e6]
            ]
        if integratorCase == 2 :
            truePos = [
                  [-2.8168016010234915e6,5.248174846916147e6,3.677157264677297e6]
                , [-6.425636528569288e6,-1.466693214251768e6,2.50438327358707e6]
                , [-2.466642497083674e6,-6.509473992136429e6,-1.6421621818735446e6]
                , [ 4.342561337924192e6,-4.1593822658140697e6,-3.947594705237753e6]
                , [ 6.279757158711852e6,2.8527385905952943e6,-1.8260959147806289e6]
            ]

        # compare the results to the truth values
        accuracy = 1.0  # meters

        testFailCount, testMessages = unitTestSupport.compareArray(
            truePos, dataPosRed, accuracy, "r_BN_N Vector",
            testFailCount, testMessages)

        #   print out success message if no error were found
        if testFailCount == 0:
            print "PASSED "
        else:
            print testFailCount
            print testMessages

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    return [testFailCount, ''.join(testMessages)]

#
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#
if __name__ == "__main__":
    run( True,       # do unit tests
         False,        # show_plots
         0            # integrator case(0 - RK4, 1 - Euler, 2 - RK2)
       )

