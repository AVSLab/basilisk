#
#  ISC License
#
#  Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
# Basilisk Scenario Script and Integrated Test
# Creation Date:
#

import os

# import non-basilisk libraries
import matplotlib.pyplot as plt
import numpy as np
# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__

from Basilisk.simulation import dualHingedRigidBodyStateEffector
# import simulation related support
from Basilisk.simulation import \
    spacecraft  # The base of any spacecraft simulation which deals with spacecraft dynamics
# import general simulation support files
from Basilisk.utilities import SimulationBaseClass  # The class which contains the basilisk simuation environment
from Basilisk.utilities import macros  # Some unit conversions
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import unitTestSupport  # general support file with common unit test functions
from Basilisk.simulation import extForceTorque
# attempt to import vizard
from Basilisk.utilities import vizSupport

bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])

def run(show_plots):
    """
    At the end of the python script you can specify the following example parameters.

    Args:
        show_plots (bool): Determines if the script should display plots
    """

    # Create simulation variable names
    dynTaskName = "dynTask"
    dynProcessName = "dynProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(dynProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.1)
    dynProcess.addTask(scSim.CreateNewTask(dynTaskName, simulationTimeStep))

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties

    # Inital spacecraft for natural dynamics without external forces and torques
    scObjectNatural = spacecraft.Spacecraft()
    scObjectNatural.ModelTag = "bsk-SatNatural"
    INatural = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObjectNatural.hub.mHub = 750  # kg - spacecraft mass
    scObjectNatural.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObjectNatural.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(INatural)

    # First sc with ext torque on the panel
    scObject1 = spacecraft.Spacecraft()
    scObject1.ModelTag = "bsk-SatExtTHinge"
    I1 = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject1.hub.mHub = 750  # kg - spacecraft mass
    scObject1.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject1.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I1)

    # Second sc with ext torque on hub
    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "bsk-Sat2"
    I2 = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject2.hub.mHub = 750  # kg - spacecraft mass
    scObject2.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject2.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I2)

    # Third sc with external force on panel
    scObject3 = spacecraft.Spacecraft()
    scObject3.ModelTag = "bsk-Sat3"
    I3 = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject3.hub.mHub = 750  # kg - spacecraft mass
    scObject3.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject3.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I3)

    # Forth sc with external force on panel
    scObject4 = spacecraft.Spacecraft()
    scObject4.ModelTag = "bsk-Sat4"
    I4 = [900., 0., 0.,
         0., 800., 0.,
         0., 0., 600.]
    scObject4.hub.mHub = 750  # kg - spacecraft mass
    scObject4.hub.r_BcB_B = [[0.0], [0.0], [0.0]]  # m - position vector of body-fixed point B relative to CM
    scObject4.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I4)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(dynTaskName, scObjectNatural)
    scSim.AddModelToTask(dynTaskName, scObject1)
    scSim.AddModelToTask(dynTaskName, scObject2)
    scSim.AddModelToTask(dynTaskName, scObject3)
    scSim.AddModelToTask(dynTaskName, scObject4)

    # Adding the HingedRigidBody State Effector
    scSim.panelNat = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    scSim.panelNat.ModelTag = "panelNat"

    scSim.panelAB = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    scSim.panelAB.ModelTag = "panelAB"

    scSim.panelCD = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    scSim.panelCD.ModelTag = "panelCD"

    scSim.panelEF = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    scSim.panelEF.ModelTag = "panelEF"

    scSim.panelGH = dualHingedRigidBodyStateEffector.DualHingedRigidBodyStateEffector()
    scSim.panelGH.ModelTag = "panelGH"

    print("here")

    # Define Variable
    scSim.panelNat.mass1 = 200
    scSim.panelNat.IPntS1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panelNat.d1 = 5
    scSim.panelNat.l1 = 10
    scSim.panelNat.k1 = 20
    scSim.panelNat.c1 = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    scSim.panelNat.r_H1B_B = [[0.5], [0.0], [1.0]]
    scSim.panelNat.dcm_H1B = [[1, 0, 0.0], [0.0, 1, 0.0], [0.0, 0.0, 1]]
    scSim.panelNat.theta1Init = 0.0
    scSim.panelNat.theta1DotInit = 0.0
    scSim.panelNat.mass2 = 200
    scSim.panelNat.IPntS2_S2 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panelNat.d2 = 5
    scSim.panelNat.k2 = 20
    scSim.panelNat.c2 = 0.0
    scSim.panelNat.theta2Init = 0.0
    scSim.panelNat.theta2DotInit = 0.0

    scSim.panelAB.mass1 = 200
    scSim.panelAB.IPntS1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panelAB.d1 = 5
    scSim.panelAB.l1 = 10
    scSim.panelAB.k1 = 20
    scSim.panelAB.c1 = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    scSim.panelAB.r_H1B_B = [[0.5], [0.0], [1.0]]
    scSim.panelAB.dcm_H1B = [[1, 0, 0.0], [0.0, 1, 0.0], [0.0, 0.0, 1]]
    scSim.panelAB.theta1Init = 0.0
    scSim.panelAB.theta1DotInit = 0.0
    scSim.panelAB.mass2 = 200
    scSim.panelAB.IPntS2_S2 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panelAB.d2 = 5
    scSim.panelAB.k2 = 20
    scSim.panelAB.c2 = 0.0
    scSim.panelAB.theta2Init = 0.0
    scSim.panelAB.theta2DotInit = 0.0

    scSim.panelCD.mass1 = 200
    scSim.panelCD.IPntS1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panelCD.d1 = 5
    scSim.panelCD.l1 = 10
    scSim.panelCD.k1 = 20
    scSim.panelCD.c1 = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    scSim.panelCD.r_H1B_B = [[0.5], [0.0], [1.0]]
    scSim.panelCD.dcm_H1B = [[1, 0, 0.0], [0.0, 1, 0.0], [0.0, 0.0, 1]]
    scSim.panelCD.theta1Init = 0.0
    scSim.panelCD.theta1DotInit = 0.0
    scSim.panelCD.mass2 = 200
    scSim.panelCD.IPntS2_S2 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panelCD.d2 = 5
    scSim.panelCD.k2 = 20
    scSim.panelCD.c2 = 0.0
    scSim.panelCD.theta2Init = 0.0
    scSim.panelCD.theta2DotInit = 0.0

    scSim.panelEF.mass1 = 200
    scSim.panelEF.IPntS1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panelEF.d1 = 5
    scSim.panelEF.l1 = 10
    scSim.panelEF.k1 = 20
    scSim.panelEF.c1 = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    scSim.panelEF.r_H1B_B = [[0.5], [0.0], [1.0]]
    scSim.panelEF.dcm_H1B = [[1, 0, 0.0], [0.0, 1, 0.0], [0.0, 0.0, 1]]
    scSim.panelEF.theta1Init = 0.0
    scSim.panelEF.theta1DotInit = 0.0
    scSim.panelEF.mass2 = 200
    scSim.panelEF.IPntS2_S2 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panelEF.d2 = 5
    scSim.panelEF.k2 = 20
    scSim.panelEF.c2 = 0.0
    scSim.panelEF.theta2Init = 0.0
    scSim.panelEF.theta2DotInit = 0.0

    scSim.panelGH.mass1 = 200
    scSim.panelGH.IPntS1_S1 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panelGH.d1 = 5
    scSim.panelGH.l1 = 10
    scSim.panelGH.k1 = 20
    scSim.panelGH.c1 = 0.0  # c is the rotational damping coefficient for the hinge, which is modeled as a spring.
    scSim.panelGH.r_H1B_B = [[0.5], [0.0], [1.0]]
    scSim.panelGH.dcm_H1B = [[1, 0, 0.0], [0.0, 1, 0.0], [0.0, 0.0, 1]]
    scSim.panelGH.theta1Init = 0.0
    scSim.panelGH.theta1DotInit = 0.0
    scSim.panelGH.mass2 = 200
    scSim.panelGH.IPntS2_S2 = [[100.0, 0.0, 0.0], [0.0, 50.0, 0.0], [0.0, 0.0, 50.0]]
    scSim.panelGH.d2 = 5
    scSim.panelGH.k2 = 20
    scSim.panelGH.c2 = 0.0
    scSim.panelGH.theta2Init = 0.0
    scSim.panelGH.theta2DotInit = 0.0

    #########
    # Add panels to spaceCraft
    #########

    # In order to affect dynamics
    scObjectNatural.addStateEffector(scSim.panelNat)
    scObject1.addStateEffector(scSim.panelAB)
    scObject2.addStateEffector(scSim.panelCD)
    scObject3.addStateEffector(scSim.panelEF)
    scObject4.addStateEffector(scSim.panelGH)

    # In order to track messages
    scSim.AddModelToTask(dynTaskName, scSim.panelAB) # in order to track messages
    scSim.AddModelToTask(dynTaskName, scSim.panelCD)
    scSim.AddModelToTask(dynTaskName, scSim.panelEF)
    scSim.AddModelToTask(dynTaskName, scSim.panelGH)

    # Setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory() # clear prior gravitational body and SPICE setup definitions
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    mu = earth.mu

    # Attach gravity model to state effector Hinge
    gravFactory.addBodiesTo(scObjectNatural)
    gravFactory.addBodiesTo(scObject1)
    gravFactory.addBodiesTo(scObject2)
    gravFactory.addBodiesTo(scObject3)
    gravFactory.addBodiesTo(scObject4)

    #########
    # setup extForceTorque module
    #########


    print("here2")

    # the control torque
    extFTObject1 = extForceTorque.ExtForceTorque()
    extFTObject1.ModelTag = "externalDisturbance"
    extFTObject1.extTorquePntB_B = [[0], [5], [0]]
    extFTObject2 = extForceTorque.ExtForceTorque()
    extFTObject2.ModelTag = "externalDisturbance"
    extFTObject2.extTorquePntB_B = [[0], [5], [0]]

    print("here 4")

    scSim.panelAB.addDynamicEffector(extFTObject1, 2) # Torque on origin of panel 3 of sc 1

    print("here 5")
    scObject2.addDynamicEffector(extFTObject2) # Torque on hub of sc 2

    scSim.AddModelToTask(dynTaskName, extFTObject1)
    scSim.AddModelToTask(dynTaskName, extFTObject2)


    # the control force
    extFTObject3 = extForceTorque.ExtForceTorque()
    extFTObject3.ModelTag = "externalDisturbance2"
    extFTObject3.extForce_B = [[0], [0], [3]]
    extFTObject4 = extForceTorque.ExtForceTorque()
    extFTObject4.ModelTag = "externalDisturbance2"
    extFTObject4.extForce_B = [[0], [0], [3]]

    scSim.panelEF.addDynamicEffector(extFTObject3, 2) # Force on origin of panel 7 of sc 3
    scObject4.addDynamicEffector(extFTObject4) # Force on hub of sc 4

    scSim.AddModelToTask(dynTaskName, extFTObject3)
    scSim.AddModelToTask(dynTaskName, extFTObject4)

    #########
    # Define mass properties of the rigid part of spacecrafts
    #########
    scObjectNatural.hub.mHub = 750.0  # kg - spacecraft mass
    scObjectNatural.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObjectNatural.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObjectNatural.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObjectNatural.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    scObject1.hub.mHub = 750.0  # kg - spacecraft mass
    scObject1.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject1.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject1.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject1.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    scObject2.hub.mHub = 750.0  # kg - spacecraft mass
    scObject2.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject2.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject2.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject2.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    scObject3.hub.mHub = 750.0  # kg - spacecraft mass
    scObject3.hub.r_BcB_B = [[0.0], [0.0], [0.0]]
    scObject3.hub.IHubPntBc_B = [[900.0, 0.0, 0.0], [0.0, 800.0, 0.0], [0.0, 0.0, 600.0]]
    scObject3.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]  # sigma_BN_B
    scObject3.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_BN_B

    # set the simulation time
    simulationTime = macros.min2nano(5)

    print("here 3")

    #########
    #   Setup data logging before the simulation is initialized
    #########
    dataLog0 = scObjectNatural.scStateOutMsg.recorder()
    dataLog1 = scObject1.scStateOutMsg.recorder()
    dataLog2 = scObject2.scStateOutMsg.recorder()
    dataLog3 = scObject3.scStateOutMsg.recorder()
    dataLog4 = scObject4.scStateOutMsg.recorder()

    print("datalog0: ", dataLog0)
    print("scSim.panelNat.dualHingedRigidBodyOutMsgs: ", scSim.panelNat.dualHingedRigidBodyOutMsgs)

    # Both panels for sc0 #TODO: CHANGE TO LETTERS
    pl0Log = scSim.panelNat.dualHingedRigidBodyOutMsgs[0].recorder()

    print("pl0Log: ", pl0Log)
    pl1Log = scSim.panelNat.dualHingedRigidBodyOutMsgs[1].recorder()
    # Both panels for sc 1
    pl2Log = scSim.panelAB.dualHingedRigidBodyOutMsgs[0].recorder()
    pl3Log = scSim.panelAB.dualHingedRigidBodyOutMsgs[1].recorder()
    # Both panels for sc 2
    pl4Log = scSim.panelCD.dualHingedRigidBodyOutMsgs[0].recorder()
    pl5Log = scSim.panelCD.dualHingedRigidBodyOutMsgs[1].recorder()
    # Both panels for sc 3
    pl6Log = scSim.panelEF.dualHingedRigidBodyOutMsgs[0].recorder()
    pl7Log = scSim.panelEF.dualHingedRigidBodyOutMsgs[1].recorder()
    # Both panels for sc 4
    pl8Log = scSim.panelGH.dualHingedRigidBodyOutMsgs[0].recorder()
    pl9Log = scSim.panelGH.dualHingedRigidBodyOutMsgs[1].recorder()

    scSim.AddModelToTask(dynTaskName, dataLog0)
    scSim.AddModelToTask(dynTaskName, dataLog1)
    scSim.AddModelToTask(dynTaskName, dataLog2)
    scSim.AddModelToTask(dynTaskName, dataLog3)
    scSim.AddModelToTask(dynTaskName, dataLog4)

    scSim.AddModelToTask(dynTaskName, pl0Log)
    scSim.AddModelToTask(dynTaskName, pl1Log)
    scSim.AddModelToTask(dynTaskName, pl2Log)
    scSim.AddModelToTask(dynTaskName, pl3Log)
    scSim.AddModelToTask(dynTaskName, pl4Log)
    scSim.AddModelToTask(dynTaskName, pl5Log)
    scSim.AddModelToTask(dynTaskName, pl6Log)
    scSim.AddModelToTask(dynTaskName, pl7Log)
    scSim.AddModelToTask(dynTaskName, pl8Log)
    scSim.AddModelToTask(dynTaskName, pl9Log)

    #########
    #   setup orbit and simulation time
    #########

    # setup the orbit using classical orbit elements
    oe = orbitalMotion.ClassicElements()
    oe.a = 10000000.0  # meters
    oe.e = 0.01
    oe.i = 33.3 * macros.D2R
    oe.Omega = 48.2 * macros.D2R
    oe.omega = 347.8 * macros.D2R
    oe.f = 85.3 * macros.D2R
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    scObjectNatural.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObjectNatural.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject1.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject1.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject2.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject2.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject3.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject3.hub.v_CN_NInit = vN  # m/s - v_CN_N
    scObject4.hub.r_CN_NInit = rN  # m   - r_CN_N
    scObject4.hub.v_CN_NInit = vN  # m/s - v_CN_N

    # if this scenario is to interface with the BSK Viz, uncomment the following lines
    # viz = vizSupport.enableUnityVisualization(scSim, dynTaskName, scObjectNatural
    #                                           , saveFile=fileName
    #                                           )

    #########
    #   initialize Simulation
    #########
    scSim.SetProgressBar(True)

    print("start init")

    scSim.InitializeSimulation()
    print("end init")

    #########
    #   configure a simulation stop time and execute the simulation run without the thruster
    #########
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()
    print("execute")

    # Hinged Rigid Body module is also set up with a message for "thetaDot" which
    # can be retrieved by replacing ".theta" with ".thetaDot".
    panel0thetaLog = pl0Log.theta
    panel1thetaLog = pl1Log.theta
    panel2thetaLog = pl2Log.theta
    panel3thetaLog = pl3Log.theta
    panel4thetaLog = pl4Log.theta
    panel5thetaLog = pl5Log.theta
    panel6thetaLog = pl6Log.theta
    panel7thetaLog = pl7Log.theta
    panel8thetaLog = pl8Log.theta
    panel9thetaLog = pl9Log.theta

    np.set_printoptions(precision=16)
    timeAxis = dataLog0.times()

    #########
    #   retrieve the logged data
    #########
    posData0 = dataLog0.r_BN_N
    velData0 = dataLog0.v_BN_N
    sigmaData0 = dataLog0.sigma_BN
    omegaData0 = np.array([np.linalg.norm(vec) for vec in dataLog0.omega_BN_B])

    posData1 = dataLog1.r_BN_N
    velData1 = dataLog1.v_BN_N
    sigmaData1 = dataLog1.sigma_BN
    omegaData1 = np.array([np.linalg.norm(vec) for vec in dataLog1.omega_BN_B])

    posData2 = dataLog2.r_BN_N
    velData2 = dataLog2.v_BN_N
    sigmaData2 = dataLog2.sigma_BN
    omegaData2 = np.array([np.linalg.norm(vec) for vec in dataLog2.omega_BN_B])

    posData3 = dataLog3.r_BN_N
    velData3 = dataLog3.v_BN_N
    sigmaData3 = dataLog3.sigma_BN
    omegaData3 = np.array([np.linalg.norm(vec) for vec in dataLog3.omega_BN_B])

    posData4 = dataLog4.r_BN_N
    velData4 = dataLog4.v_BN_N
    sigmaData4 = dataLog4.sigma_BN
    omegaData4 = np.array([np.linalg.norm(vec) for vec in dataLog4.omega_BN_B])

    angleHistory0 = []
    angleHistory1 = []
    angleHistory2 = []
    angleHistory3 = []
    angleHistory4 = []
    angleHistory5 = []
    angleHistory6 = []
    angleHistory7 = []
    angleHistory8 = []
    angleHistory9 = []

    for i in range(len(timeAxis)):
        angle0 = 4 * np.arctan(np.linalg.norm(sigmaData0[i, :]))
        angleHistory0.append(angle0)

    for i in range(len(timeAxis)):
        angle = 4 * np.arctan(np.linalg.norm(sigmaData1[i, :]))
        angleHistory1.append(angle)

    for i in range(len(timeAxis)):
        angle2 = 4 * np.arctan(np.linalg.norm(sigmaData2[i, :]))
        angleHistory2.append(angle2)

    for i in range(len(timeAxis)):
        angle3 = 4 * np.arctan(np.linalg.norm(sigmaData3[i, :]))
        angleHistory3.append(angle3)

    for i in range(len(timeAxis)):
        angle4 = 4 * np.arctan(np.linalg.norm(sigmaData4[i, :]))
        angleHistory4.append(angle4)

    print("start plots")

    #########
    #   plot the results
    #########
    # draw the inertial position vector components
    plt.close("all")  # clears out plots from earlier test runs

    # plt.figure(1)
    # fig = plt.gcf()
    # ax = fig.gca()
    # ax.ticklabel_format(useOffset=False, style='plain')
    # for idx in range(3):
    #     plt.plot(timeAxis * macros.NANO2MIN, posData0[:, idx] / 1000.,
    #              color=unitTestSupport.getLineColor(idx, 3),
    #              label='Natural Dynamics: $r_{BN,' + str(idx) + '}$')
    # for idx in range(3):
    #     plt.plot(timeAxis * macros.NANO2MIN, posData1[:, idx] / 1000.,
    #              color=unitTestSupport.getLineColor(idx, 3),
    #              label='External Torque on HRB: $r_{BN,' + str(idx) + '}$')
    # for idx in range(3):
    #     plt.plot(timeAxis * macros.NANO2MIN, posData2[:, idx] / 1000.,
    #              color=unitTestSupport.getLineColor(idx, 3),
    #              label='External Torque on hub: $r_{BN,' + str(idx) + '}$')
    # plt.legend(loc='lower right')
    # plt.xlabel('Time [h]')
    # plt.ylabel('Inertial Position [km]')
    # plt.title("Inertial Position of SC Subject to External Torque")
    # figureList = {}
    # pltName = fileName + "1" + str(int(0.))
    # figureList[pltName] = plt.figure(1)

    # plt.figure(2)
    # fig = plt.gcf()
    # ax = fig.gca()
    # ax.ticklabel_format(useOffset=False, style='plain')
    # rData0 = []
    # rData1 = []
    # rData2 = []
    # for idx in range(0, len(posData0)):
    #     oeData0 = orbitalMotion.rv2elem_parab(mu, posData0[idx], velData0[idx])
    #     rData0.append(oeData0.rmag / 1000.)
    # for idx in range(0, len(posData3)):
    #     oeData1 = orbitalMotion.rv2elem_parab(mu, posData1[idx], velData1[idx])
    #     rData1.append(oeData1.rmag / 1000.)
    # plt.plot(timeAxis * macros.NANO2MIN, rData3,
    #          )
    # for idx in range(0, len(posData4)):
    #     oeData2 = orbitalMotion.rv2elem_parab(mu, posData2[idx], velData2[idx])
    #     rData2.append(oeData2.rmag / 1000.)
    # plt.plot(timeAxis * macros.NANO2MIN, rData0, label = "Natural Dynamics",
    #          )
    # plt.plot(timeAxis * macros.NANO2MIN, rData1, label = "Torque on HRB",
    #          )
    # plt.plot(timeAxis * macros.NANO2MIN, rData2, label = "Torque on Hub",
    #          )
    # plt.xlabel('Time [min]')
    # plt.ylabel('Radius [km]')
    # plt.title("Radius wrt Earth with SC Subject to External Torque")
    # figureList = {}
    # pltName = fileName + "2" + str(int(0.))
    # figureList[pltName] = plt.figure(2)

    # SC1 (panels 2+3): Ext Torque is on hub vs SC2 (panels 4+5): Ext Torque on Panel 5's origin
    plt.figure(3)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    plt.plot(timeAxis * macros.NANO2MIN, panel0thetaLog, linestyle=':', label= "Natural dynamics: Panel 1 theta[r]")
    plt.plot(timeAxis * macros.NANO2MIN, panel1thetaLog, linestyle=':', label= "Natural dynamics: Panel 2 theta[r]")

    plt.plot(timeAxis * macros.NANO2MIN, panel2thetaLog, linestyle=':', label= "Torque on 2nd panel of HRB: Panel 1 theta[r]")
    plt.plot(timeAxis * macros.NANO2MIN, panel3thetaLog, linestyle='-.', label= "Torque on 2nd panel of HRB: Panel 2 theta[r]")

    plt.plot(timeAxis * macros.NANO2MIN, panel4thetaLog, linestyle='--', label= "Torque on Hub: Panel 1 theta[r]")
    plt.plot(timeAxis * macros.NANO2MIN, panel5thetaLog, linestyle='--', label= "Torque on Hub: Panel 2 theta[r]")

    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Angular Displacement [r]')
    plt.title("Panel Angular Displacement wrt Hub due to External Torque")
    pltName = fileName + "panelABtheta and panel2theta" + str(int(0.))
    figureList = {}
    figureList[pltName] = plt.figure(3)


    # SC3 (panels 6+7): Ext Force is on hub vs SC4 (panels 8+9): Ext Force on Panel 9's origin
    plt.figure(4)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    plt.plot(timeAxis * macros.NANO2MIN, panel0thetaLog, linestyle='--', label= "Natural dynamics: Panel 1 theta[r]")
    plt.plot(timeAxis * macros.NANO2MIN, panel1thetaLog, linestyle='-.', label= "Natural dynamics: Panel 2 theta[r]")

    plt.plot(timeAxis * macros.NANO2MIN, panel6thetaLog, linestyle='-.', label= "Force on 2nd panel of HRB: Panel 1 theta[r]")
    plt.plot(timeAxis * macros.NANO2MIN, panel7thetaLog, linestyle='-.', label= "Force on 2nd panel of HRB: Panel 2 theta[r]")

    plt.plot(timeAxis * macros.NANO2MIN, panel8thetaLog, linestyle='--', label= "Force on Hub: Panel 1 theta [r]")
    plt.plot(timeAxis * macros.NANO2MIN, panel9thetaLog, linestyle='--', label= "Force on Hub: Panel 2 theta[r]")

    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Angular Displacement [r]')
    plt.title("Panel Angular Displacement wrt Hub due to External Force")
    pltName = fileName + "panelABtheta and panel2theta" + str(int(0.))
    figureList = {}
    figureList[pltName] = plt.figure(4)



    # plt.figure(5)
    # fig = plt.gcf()
    # ax = fig.gca()
    # ax.ticklabel_format(useOffset=False, style='plain')
    # plt.plot(timeAxis * macros.NANO2MIN, angleHistory0, label="Natural dynamics")
    # plt.plot(timeAxis * macros.NANO2MIN, angleHistory1, label="Torque on HRB")
    # plt.plot(timeAxis * macros.NANO2MIN, angleHistory2, label="Torque on Hub")
    # plt.legend(loc='lower right')
    # plt.xlabel('Time [min]')
    # plt.ylabel('Angular Displacement [r]')
    # plt.title("Panel Angular Displacement wrt Earth due to External Torque")
    # pltName = fileName + "angleData" + str(int(0.))
    # figureList[pltName] = plt.figure(5)


    ########################################################################################
    # Plot only ext force:
    ########################################################################################

    # Plots to compare when ExtForce is on hub vs panel
    # plt.figure(6)
    # fig = plt.gcf()
    # ax = fig.gca()
    # ax.ticklabel_format(useOffset=False, style='plain')
    # for idx in range(3):
    #     plt.plot(timeAxis * macros.NANO2MIN, posData1[:, idx] / 1000.,
    #              color=unitTestSupport.getLineColor(idx, 3),
    #              label='Force on panel: $r_{BN,' + str(idx) + '}$')
    # for idx in range(3):
    #     plt.plot(timeAxis * macros.NANO2MIN, posData2[:, idx] / 1000.,
    #              color=unitTestSupport.getLineColor(idx, 3),
    #              label='Force on hub: $r_{BN,' + str(idx) + '}$')
    # plt.legend(loc='lower right')
    # plt.xlabel('Time [h]')
    # plt.ylabel('Inertial Position [km]')
    # figureList = {}
    # pltName = fileName + "1" + str(int(0.))
    # figureList[pltName] = plt.figure(6)

    # plt.figure(7)
    # fig = plt.gcf()
    # ax = fig.gca()
    # ax.ticklabel_format(useOffset=False, style='plain')
    # rData0 = []
    # rData3 = []
    # rData4 = []
    # for idx in range(0, len(posData0)):
    #     oeData0 = orbitalMotion.rv2elem_parab(mu, posData0[idx], velData0[idx])
    #     rData0.append(oeData0.rmag / 1000.)
    # for idx in range(0, len(posData4)):
    #     oeData1 = orbitalMotion.rv2elem_parab(mu, posData3[idx], velData3[idx])
    #     rData3.append(oeData1.rmag / 1000.)
    # for idx in range(0, len(posData3)):
    #     oeData2 = orbitalMotion.rv2elem_parab(mu, posData4[idx], velData4[idx])
    #     rData4.append(oeData2.rmag / 1000.)
    # plt.plot(timeAxis * macros.NANO2MIN, rData0, label = "Natural Dynamics",
    #          )
    # plt.plot(timeAxis * macros.NANO2MIN, rData3, label = "Force on HRB",
    #          )
    # plt.plot(timeAxis * macros.NANO2MIN, rData4, label = "Force on Hub",
    #          )
    # plt.legend(loc='lower right')
    # plt.xlabel('Time [min]')
    # plt.ylabel('Radius [km]')
    # plt.title("Radius wrt Earth with SC Subject to External Force")
    # pltName = fileName + "2" + str(int(0.))
    # figureList = {}
    # figureList[pltName] = plt.figure(7)

    # Plots to compare with ExtForce
    # plt.figure(8)
    # fig = plt.gcf()
    # ax = fig.gca()
    # ax.ticklabel_format(useOffset=False, style='plain')
    # plt.plot(timeAxis * macros.NANO2MIN, panel0thetaLog, label="Natural dynamics")
    # plt.plot(timeAxis * macros.NANO2MIN, panel3thetaLog,
    #          label="Force on HRB",
    #          color='orange',
    #          linestyle='--',
    #          )
    # plt.plot(timeAxis * macros.NANO2MIN, panel4thetaLog,
    #         label="Force on Hub",
    #         color='green',
    #         linestyle=':',
    #         )
    # plt.legend(loc='lower right')
    # plt.xlabel('Time [min]')
    # plt.ylabel('Angular Displacement [r]')
    # plt.title("Panel Angular Displacement wrt hub due to External Force")
    # pltName = fileName + "Panel theta" + str(int(0.))
    # figureList[pltName] = plt.figure(8)

    # plt.figure(9)
    # fig = plt.gcf()
    # ax = fig.gca()
    # ax.ticklabel_format(useOffset=False, style='plain')
    # plt.plot(timeAxis * macros.NANO2MIN, angleHistory0, label = "Natural dynamics")
    # plt.plot(timeAxis * macros.NANO2MIN, angleHistory3, label = "Force on HRB")
    # plt.plot(timeAxis * macros.NANO2MIN, angleHistory4, linestyle='--', label = "Force on Hub")
    # plt.legend(loc='lower right')
    # plt.xlabel('Time [min]')
    # plt.ylabel('Angular Displacement [r]')
    # plt.title("Hub Angular Displacement wrt Earth due to External Force")
    # pltName = fileName + "angleData" + str(int(0.))
    # figureList[pltName] = plt.figure(9)

    # Plot omega of hub
    plt.figure(10)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    plt.plot(timeAxis * macros.NANO2MIN, omegaData0, label= "Natural dynamics")
    plt.plot(timeAxis * macros.NANO2MIN, omegaData1, label= "External torque on HRB")
    plt.plot(timeAxis * macros.NANO2MIN, omegaData2, label= "External torque on hub")
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Angular velocity [r]')
    plt.title("Angular Velocity of hub wrt Earth due to torque")
    pltName = fileName + "angleData" + str(int(0.))
    figureList[pltName] = plt.figure(10)

    # Plot omega of hub
    plt.figure(11)
    fig = plt.gcf()
    ax = fig.gca()
    ax.ticklabel_format(useOffset=False, style='plain')
    plt.plot(timeAxis * macros.NANO2MIN, omegaData0, label= "Natural dynamics")
    plt.plot(timeAxis * macros.NANO2MIN, omegaData3, linestyle=':', label= "External force at origin of H frame")
    plt.plot(timeAxis * macros.NANO2MIN, omegaData4, label= "External force at origin of B frame")
    plt.legend(loc='lower right')
    plt.xlabel('Time [min]')
    plt.ylabel('Angular velocity [r]')
    plt.title("Angular Velocity of hub wrt Earth due to force")
    pltName = fileName + "angleData" + str(int(0.))
    figureList[pltName] = plt.figure(11)

    if show_plots:
        plt.show()

    # close the plots being saved off to avoid over-writing old and new figures
    plt.close("all")

    return velData1, figureList

#########
# This statement below ensures that the unit test scrip can be run as a
# stand-along python script
#########
if __name__ == "__main__":
    run(
        True  # show_plots
    )
