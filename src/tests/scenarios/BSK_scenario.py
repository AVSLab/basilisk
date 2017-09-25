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
import sys, os, inspect

import BSK_main
import BSK_plotting as BSKPlt
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import macros as mc
from Basilisk.utilities import unitTestSupport as sp


if __name__ == "__main__":
    TheBSKSim = BSK_main.BSKSim()

    # Log data for post-processing and plotting
    simulationTime = mc.min2nano(10.)
    numDataPoints = 100
    samplingTime = simulationTime / (numDataPoints-1)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.FSWClass.mrpFeedbackData.outputDataName, samplingTime)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.FSWClass.trackingErrorData.outputDataName, samplingTime)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.DynClass.simpleNavObject.outputTransName, samplingTime)
    TheBSKSim.TotalSim.logThisMessage(TheBSKSim.DynClass.simpleNavObject.outputAttName, samplingTime)

    # Initialize Spacecraft States with the initialization variables

    # Set up the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    mu = TheBSKSim.DynClass.earthGravBody.mu
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * mc.D2R
    oe.Omega = 48.2 * mc.D2R
    oe.omega = 347.8 * mc.D2R
    oe.f = 85.3 * mc.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)

    TheBSKSim.scObject.hub.r_CN_NInit =  sp.np2EigenVectorXd(rN)  # r_BN_N [m]
    TheBSKSim.scObject.hub.v_CN_NInit = sp.np2EigenVectorXd(vN)  # r_BN_N [m]
    TheBSKSim.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    TheBSKSim.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # omega_BN_B [rad/s]

    # Initialize Simulation
    TheBSKSim.InitializeSimulation()
    # The next call ensures that the FSW and Dynamics Message that have the same
    # name are copied over every time the simulation ticks forward.  This function
    # has to be called after the simulation is initialized to ensure that all modules
    # have created their own output/input messages declarations.
    TheBSKSim.dyn2FSWInterface.discoverAllMessages()
    TheBSKSim.fsw2DynInterface.discoverAllMessages()


    # Configure a simulation stop time time and execute the simulation run
    TheBSKSim.modeRequest = 'hillPoint'
    TheBSKSim.ConfigureStopTime(simulationTime)
    TheBSKSim.ExecuteSimulation()


    # Retrieve the logged data
    dataLr = TheBSKSim.pullMessageLogData(TheBSKSim.FSWClass.mrpFeedbackData.outputDataName + ".torqueRequestBody", range(3))
    dataSigmaBR = TheBSKSim.pullMessageLogData(TheBSKSim.FSWClass.trackingErrorData.outputDataName + ".sigma_BR", range(3))
    dataOmegaBR = TheBSKSim.pullMessageLogData(TheBSKSim.FSWClass.trackingErrorData.outputDataName + ".omega_BR_B", range(3))
    dataPos = TheBSKSim.pullMessageLogData(TheBSKSim.DynClass.simpleNavObject.outputTransName + ".r_BN_N", range(3))
    dataVel = TheBSKSim.pullMessageLogData(TheBSKSim.DynClass.simpleNavObject.outputTransName + ".v_BN_N", range(3))
    dataSigmaBN = TheBSKSim.pullMessageLogData(TheBSKSim.DynClass.simpleNavObject.outputAttName + ".sigma_BN", range(3))


    BSKPlt.plotResults(dataLr, dataSigmaBR, dataOmegaBR, dataPos, dataVel, dataSigmaBN)
