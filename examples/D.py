#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

import os
import numpy as np
import matplotlib.pyplot as plt
from copy import copy

from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

from Basilisk.utilities import (SimulationBaseClass, macros, orbitalMotion,
simIncludeGravBody, unitTestSupport, vizSupport)

from Basilisk.simulation import spacecraft, dentonFluxModel
from Basilisk.architecture import messaging

def run(show_plots):
    """
    Illustration of adding Basilisk modules to a task
    """

    # Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # See progress bar
    scSim.SetProgressBar(True)
    
    # Create the simulation process
    simProcessName = "dynamicsProcess"
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # Create the dynamics task and specify the integration update time
    simTaskName = "dynamicsTask"
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, macros.sec2nano(10.)))
    
    # Setup the simulation tasks/objects
    # Initialize spacecraft object and set properties
    # The dynamics simulation is setup using a Spacecraft() module.
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "bsk-Sat"
    
    # Add spacecraft obect to the simulation process
    scSim.AddModelToTask(simTaskName, scObject)
    
    
    ################################################################################################################################
    
    gravFactory = simIncludeGravBody.gravBodyFactory()
        
    # Setup Earth gravity body
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    mu = earth.mu
    
    # Setup Sun gravity body
    sun = gravFactory.createSun()
    
    # Only add Earth as a gravity body
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values("0")))
    
    # Next, the default SPICE support module is created and configured.  The first step is to store
    # the date and time of the start of the simulation.
    timeInitString = "2012 MAY 1 00:28:30.0"
    spiceTimeStringFormat = '%Y %B %d %H:%M:%S.%f'
    timeInit = datetime.strptime(timeInitString, spiceTimeStringFormat)

    # The following is a support macro that creates a `gravFactory.spiceObject` instance, and fills in typical
    # default parameters.  By setting the epochInMsg argument, this macro provides an epoch date/time
    # message as well.  The spiceObject is set to subscribe to this epoch message.  Using the epoch message
    # makes it trivial to synchronize the epoch information across multiple modules.
    gravFactory.createSpiceInterface(bskPath +'/supportData/EphemerisData/',
                                     timeInitString,
                                     epochInMsg=True)

    # By default the SPICE object will use the solar system barycenter as the inertial origin
    # If the spacecraft() output is desired relative to another celestial object, the zeroBase string
    # name of the SPICE object needs to be changed.
    # Next the SPICE module is customized.  The first step is to specify the zeroBase.  This is the inertial
    # origin relative to which all spacecraft message states are taken.  The simulation defaults to all
    # planet or spacecraft ephemeris being given in the SPICE object default frame, which is the solar system barycenter
    # or SSB for short.  The spacecraft() state output message is relative to this SBB frame by default.  To change
    # this behavior, the zero based point must be redefined from SBB to another body.
    # In this simulation we use the Earth.
    gravFactory.spiceObject.zeroBase = 'Earth'

    # Finally, the SPICE object is added to the simulation task list.
    scSim.AddModelToTask(simTaskName, gravFactory.spiceObject)
    
    ################################################################################################################################
    
    # Setup orbit and simulation time
    #
    # Setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    rLEO = 7000. * 1000      # meters
    rGEO = 42000. * 1000     # meters
    oe.a = rGEO
    oe.e = 0.00001
    oe.i = 0.0 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    oe = orbitalMotion.rv2elem(mu, rN, vN)      # This stores consistent initial orbit elements

    # To set the spacecraft initial conditions, the following initial position and velocity variables are set:
    scObject.hub.r_CN_NInit = rN  # m   - r_BN_N
    scObject.hub.v_CN_NInit = vN  # m/s - v_BN_N
    
    # set the simulation time
    n = np.sqrt(mu / oe.a / oe.a / oe.a)
    P = 2. * np.pi / n
    simulationTime = macros.sec2nano(P)
    
    numDataPoints = 200
    samplingTime = unitTestSupport.samplingTime(simulationTime, 10, numDataPoints)
    #msgRec = scObject.scStateOutMsg.recorder(samplingTime)
    #scSim.AddModelToTask(simTaskName, msgRec)
    
    # Create copies of the Basilisk modules
    mod1 = dentonFluxModel.DentonFluxModel()
    mod1.ModelTag = "cppModule"
    scSim.AddModelToTask(simTaskName, mod1, None, 10)

    ##########################################################################################################
    
    # Connect messages
    mod1.satStateInMsg.subscribeTo(scObject.scStateOutMsg);
    # mod1.sunStateInMsg.subscribeTo(sun.planetStateOutMsgs);
    mod1.sunStateInputMsg.subscribeTo(gravFactory.gravBodies.values.planetStateOutMsgs("0"));

    ##########################################################################################################

    
    # Setup message recording
    msgRec = mod1.fluxOutMsg.recorder(samplingTime)
    scSim.AddModelToTask(simTaskName, msgRec)
    
    # Set Desired Energy and Kp From Script (FOR NOW)
    mod1.choose_kp = 2;
    mod1.choose_energy = 2400;
    
    # Initialize Simulation:
    scSim.InitializeSimulation()

    # Configure a simulation stop time time and execute the simulation run
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()
    
    # Retrieve logged data
    meanElecFlux = msgRec.meanElectronFlux;
    meanIonFlux = msgRec.meanIonFlux;
    timeAxis = msgRec.times()
    print(meanElecFlux)
    np.set_printoptions(precision=16)
        
    # Plot retrieved data
    #figureList = plotOrbits(timeAxis, posData, velData, msgRec, oe, P, mu, planet)
    
    #if show_plots:
        #plt.show()
    
    #plt.close("all")
    
    return 0;
    
    #return figureList
    
def plotOrbits(timeAxis, posData, velData, msgRec, oe, P, mu, planet):
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs
    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    finalDiff = 0.0

    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2SEC / P, posData[:, idx] / 1000.,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label='$r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Inertial Position [km]')
    figureList = {}
    pltName = fileName + "1"
    figureList[pltName] = plt.figure(1)

    # draw orbit in perifocal frame
    b = oe.a * np.sqrt(1 - oe.e * oe.e)
    p = oe.a * (1 - oe.e * oe.e)
    plt.figure(2, figsize=np.array((1.0, b / oe.a)) * 4.75, dpi=100)
    plt.axis(np.array([-oe.rApoap, oe.rPeriap, -b, b]) / 1000 * 1.25)
    # draw the planet
    fig = plt.gcf()
    ax = fig.gca()
    planetColor = '#008800'
    planetRadius = planet.radEquator / 1000
    ax.add_artist(plt.Circle((0, 0), planetRadius, color=planetColor))
    # draw the actual orbit
    rData = []
    fData = []
    for idx in range(0, len(posData)):
        oeData = orbitalMotion.rv2elem(mu, posData[idx], velData[idx])
        rData.append(oeData.rmag)
        fData.append(oeData.f + oeData.omega - oe.omega)
    plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, color='#aa0000', linewidth=3.0
             )
    # draw the full osculating orbit from the initial conditions
    #fData = np.linspace(0, 2 * np.pi, 100)
    #rData = []
    #for idx in range(0, len(fData)):
        #rData.append(p / (1 + oe.e * np.cos(fData[idx])))
    #plt.plot(rData * np.cos(fData) / 1000, rData * np.sin(fData) / 1000, '--', color='#555555'
             #)
    #plt.xlabel('$i_e$ Cord. [km]')
    #plt.ylabel('$i_p$ Cord. [km]')
    #plt.grid()

    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    Deltar = np.empty((0, 3))
    E0 = orbitalMotion.f2E(oe.f, oe.e)
    M0 = orbitalMotion.E2M(E0, oe.e)
    n = np.sqrt(mu/(oe.a*oe.a*oe.a))
    oe2 = copy(oe)
    for idx in range(0, len(posData)):
        M = M0 + n * timeAxis[idx] * macros.NANO2SEC
        Et = orbitalMotion.M2E(M, oe.e)
        oe2.f = orbitalMotion.E2f(Et, oe.e)
        rv, vv = orbitalMotion.elem2rv(mu, oe2)
        Deltar = np.append(Deltar, [posData[idx] - rv], axis=0)
    for idx in range(3):
        plt.plot(timeAxis * macros.NANO2SEC / P, Deltar[:, idx] ,
                 color=unitTestSupport.getLineColor(idx, 3),
                 label=r'$\Delta r_{BN,' + str(idx) + '}$')
    plt.legend(loc='lower right')
    plt.xlabel('Time [orbits]')
    plt.ylabel('Trajectory Differences [m]')
    pltName = fileName + "2"
    figureList[pltName] = plt.figure(2)

    #finalDiff = np.linalg.norm(Deltar[-1])
    
    return figureList

if __name__ == "__main__":
    run(
        True,        # show_plots
    )

