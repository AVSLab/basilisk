'''
 ISC License

 Copyright (c) 2020, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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


import numpy as np
import os
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.animation as animation
from Basilisk.simulation import RigidBodyContactEffector, spacecraft
from Basilisk.utilities import (SimulationBaseClass, macros, RigidBodyKinematics, unitTestSupport, vizSupport)
from Basilisk.architecture import messaging
from Basilisk.simulation import svIntegrators
import copy
# import GeometryUtility as GEO

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def create_vert_data(rN, sigma):
    C = RigidBodyKinematics.MRP2C(-sigma)
    # Vertices
    v1 = rN + (C @ np.asarray([-0.5, -0.5, 0.5]))
    v2 = rN + (C @ np.asarray([0.5, -0.5, 0.5]))
    v3 = rN + (C @ np.asarray([-0.5, 0.5, 0.5]))
    v4 = rN + (C @ np.asarray([0.5, 0.5, 0.5]))
    v5 = rN + (C @ np.asarray([-0.5, 0.5, -0.5]))
    v6 = rN + (C @ np.asarray([0.5, 0.5, -0.5]))
    v7 = rN + (C @ np.asarray([-0.5, -0.5, -0.5]))
    v8 = rN + (C @ np.asarray([0.5, -0.5, -0.5]))
    # Face 1
    x = [v1[0], v2[0], v3[0]]
    y = [v1[1], v2[1], v3[1]]
    z = [v1[2], v2[2], v3[2]]
    verts1 = [list(zip(x,y,z))]
    # Face 2
    x = [v3[0], v2[0], v4[0]]
    y = [v3[1], v2[1], v4[1]]
    z = [v3[2], v2[2], v4[2]]
    verts2 = [list(zip(x, y, z))]
    # Face 3
    x = [v3[0], v4[0], v5[0]]
    y = [v3[1], v4[1], v5[1]]
    z = [v3[2], v4[2], v5[2]]
    verts3 = [list(zip(x, y, z))]
    # Face 4
    x = [v5[0], v4[0], v6[0]]
    y = [v5[1], v4[1], v6[1]]
    z = [v5[2], v4[2], v6[2]]
    verts4 = [list(zip(x, y, z))]
    # Face 5
    x = [v5[0], v6[0], v7[0]]
    y = [v5[1], v6[1], v7[1]]
    z = [v5[2], v6[2], v7[2]]
    verts5 = [list(zip(x, y, z))]
    # Face 6
    x = [v7[0], v6[0], v8[0]]
    y = [v7[1], v6[1], v8[1]]
    z = [v7[2], v6[2], v8[2]]
    verts6 = [list(zip(x, y, z))]
    # Face 7
    x = [v7[0], v8[0], v1[0]]
    y = [v7[1], v8[1], v1[1]]
    z = [v7[2], v8[2], v1[2]]
    verts7 = [list(zip(x, y, z))]
    # Face 8
    x = [v1[0], v8[0], v2[0]]
    y = [v1[1], v8[1], v2[1]]
    z = [v1[2], v8[2], v2[2]]
    verts8 = [list(zip(x, y, z))]
    # Face 9
    x = [v2[0], v8[0], v4[0]]
    y = [v2[1], v8[1], v4[1]]
    z = [v2[2], v8[2], v4[2]]
    verts9 = [list(zip(x, y, z))]
    # Face 10
    x = [v4[0], v8[0], v6[0]]
    y = [v4[1], v8[1], v6[1]]
    z = [v4[2], v8[2], v6[2]]
    verts10 = [list(zip(x, y, z))]
    # Face 11
    x = [v7[0], v1[0], v5[0]]
    y = [v7[1], v1[1], v5[1]]
    z = [v7[2], v1[2], v5[2]]
    verts11 = [list(zip(x, y, z))]
    # Face 12
    x = [v5[0], v1[0], v3[0]]
    y = [v5[1], v1[1], v3[1]]
    z = [v5[2], v1[2], v3[2]]
    verts12 = [list(zip(x, y, z))]

    verts = [verts1, verts2, verts3, verts4, verts5, verts6, verts7, verts8, verts9, verts10, verts11, verts12]

    return verts

def update_shapes(frame, cube1, cube2, shapes):
    cubes = cube1[frame]
    cubes.extend(cube2[frame])
    for shape, cube in zip(shapes, cubes):
        shape.set_verts(cube)
        shape.set_edgecolor('k')
    return shapes



def run():

    # Create simulation variable names
    simTaskName = "simTask"
    simProcessName = "simProcess"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    # set the simulation time variable used later on
    simulationTime = macros.min2nano(0.02)

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(.01)
    dynProcess.addTask(scSim.CreateNewTask(simTaskName, simulationTimeStep))

    # initialize object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "Primary"
    integratorObject = svIntegrators.svIntegratorRKF45(scObject)
    scObject.setIntegrator(integratorObject)
    # define the simulation inertia
    # mMainBody = 95.0
    # xDimMainBody = 2.0
    # yDimMainBody = 1.0
    # zDimMainBody = 0.6
    # posMainBody = np.array([0.0, 0.0, 0.2])
    # IMainBody = np.array([[((1. / 12.) * mMainBody * ((yDimMainBody ** 2.) + (zDimMainBody ** 2.))), 0., 0.],
    #      [0., ((1. / 12.) * mMainBody * ((xDimMainBody ** 2.) + (zDimMainBody ** 2.))), 0.],
    #      [0., 0., ((1. / 12.) * mMainBody * ((xDimMainBody ** 2.) + (yDimMainBody ** 2.)))]]) - mMainBody * np.array(RigidBodyKinematics.v3Tilde(-posMainBody)) @ np.array(
    #     RigidBodyKinematics.v3Tilde(-posMainBody)).transpose()
    #
    # mLeftLeg = 5.0
    # xLeftLeg = 1.0
    # yLeftLeg = 0.2
    # zLeftLeg = 0.2
    # posLeftLeg = np.array([1.0, 0, -0.3])
    # dcmLeftLeg = RigidBodyKinematics.Mi(np.deg2rad(45.), 2)
    # ILeftLeg = dcmLeftLeg @ np.array([[((1. / 12.) * mLeftLeg * ((yLeftLeg ** 2.) + (zLeftLeg ** 2.))), 0., 0.],
    #      [0., ((1. / 12.) * mLeftLeg * ((xLeftLeg ** 2.) + (zLeftLeg ** 2.))), 0.],
    #      [0., 0., ((1. / 12.) * mLeftLeg * ((xLeftLeg ** 2.) + (yLeftLeg ** 2.)))]]) @ dcmLeftLeg.transpose() \
    #            + mLeftLeg * np.array(RigidBodyKinematics.v3Tilde(-posLeftLeg)) @ np.array(
    #     RigidBodyKinematics.v3Tilde(-posLeftLeg)).transpose()
    #
    # mRightLeg = 5.0
    # xRightLeg = 1.0
    # yRightLeg = 0.2
    # zRightLeg = 0.2
    # posRightLeg = np.array([-1.0, 0, -0.3])
    # dcmRightLeg = RigidBodyKinematics.Mi(np.deg2rad(-45.), 2)
    # IRightLeg = dcmRightLeg @ np.array([[((1. / 12.) * mRightLeg * ((yRightLeg ** 2.) + (zRightLeg ** 2.))), 0., 0.],
    #                                   [0., ((1. / 12.) * mRightLeg * ((xRightLeg ** 2.) + (zRightLeg ** 2.))), 0.],
    #                                   [0., 0., ((1. / 12.) * mRightLeg * (
    #                                               (xRightLeg ** 2.) + (yRightLeg ** 2.)))]]) @ dcmRightLeg.transpose() \
    #            + mRightLeg * np.array(RigidBodyKinematics.v3Tilde(-posRightLeg)) @ np.array(
    #     RigidBodyKinematics.v3Tilde(-posRightLeg)).transpose()
    #
    # mLeftFoot = 10.0
    # xLeftFoot = 1.0
    # yLeftFoot = 1.4
    # zLeftFoot = 0.4
    # posLeftFoot = np.array([1.5, 0, -0.8])
    # ILeftFoot = np.array([[((1. / 12.) * mLeftFoot * ((yLeftFoot ** 2.) + (zLeftFoot ** 2.))), 0., 0.],
    #                                     [0., ((1. / 12.) * mLeftFoot * ((xLeftFoot ** 2.) + (zLeftFoot ** 2.))), 0.],
    #                                     [0., 0., ((1. / 12.) * mLeftFoot * (
    #                                             (xLeftFoot ** 2.) + (yLeftFoot ** 2.)))]]) \
    #             + mLeftFoot * np.array(RigidBodyKinematics.v3Tilde(-posLeftFoot)) @ np.array(
    #     RigidBodyKinematics.v3Tilde(-posLeftFoot)).transpose()
    #
    # mRightFoot = 10.0
    # xRightFoot = 1.0
    # yRightFoot = 1.4
    # zRightFoot = 0.4
    # posRightFoot = np.array([-1.5, 0, -0.8])
    # IRightFoot = np.array([[((1. / 12.) * mRightFoot * ((yRightFoot ** 2.) + (zRightFoot ** 2.))), 0., 0.],
    #                       [0., ((1. / 12.) * mRightFoot * ((xRightFoot ** 2.) + (zRightFoot ** 2.))), 0.],
    #                       [0., 0., ((1. / 12.) * mRightFoot * (
    #                               (xRightFoot ** 2.) + (yRightFoot ** 2.)))]]) \
    #             + mRightFoot * np.array(RigidBodyKinematics.v3Tilde(-posRightFoot)) @ np.array(
    #     RigidBodyKinematics.v3Tilde(-posRightFoot)).transpose()
    #
    # CoMLander = (mMainBody * posMainBody + mLeftLeg * posLeftLeg + mRightLeg * posRightLeg + mLeftFoot * posLeftFoot + mRightFoot * posRightFoot) / (
    #     mMainBody + mRightFoot + mLeftFoot + mLeftLeg + mRightLeg)

    scObject.hub.mHub = 5.0  # kg - spacecraft mass
    I = [((1./6.)*scObject.hub.mHub*4.), 0., 0.,
         0., ((1./6.)*scObject.hub.mHub*4.), 0.,
         0., 0., ((1./6.)*scObject.hub.mHub*4.)]

    # I = [((1. / 6.) * scObject.hub.mHub * 9.), 0., 0.,
    #      0., ((1. / 6.) * scObject.hub.mHub * 9.), 0.,
    #      0., 0., ((1. / 6.) * scObject.hub.mHub * 9.)]

    # IAll = IMainBody + ILeftFoot + ILeftLeg + IRightFoot + IRightLeg
    # I = [IAll[0, 0], IAll[0, 1], IAll[0, 2],
    #      IAll[1, 0], IAll[1, 1], IAll[1, 2],
    #      IAll[2, 0], IAll[2, 1], IAll[2, 2]]
    # scObject.hub.mHub = mMainBody + mRightLeg + mLeftLeg + mLeftFoot + mRightFoot
    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)



    scSim.AddModelToTask(simTaskName, scObject, None, 1)

    scContact = RigidBodyContactEffector.RigidBodyContactEffector()
    scContact.maxPosError = 1e-4
    scContact.simTimeStep = 0.01
    scContact.slipTolerance = 1e-6
    scContact.collisionIntegrationStep = 1e-4
    scContact.maxBoundingBoxDim = 0.7
    scContact.minBoundingBoxDim = 0.01
    scContact.boundingBoxFF = 1
    scContact.LoadSpacecraftBody("cube2.obj", scObject.ModelTag, scObject.scStateOutMsg, scObject.scMassOutMsg, 1.0, 1.0, 0.0)
    # scContact.LoadMainBody("Lander.obj")
    # scContact.mainBody.modelTag = scObject.ModelTag
    # scContact.mainBody.boundingRadius = 3.5
    # scContact.mainBody.boundingRadius = 1.5


    staticObjectMsg = messaging.SpicePlanetStateMsgPayload()
    # staticObjectMsg.PositionVector = [3.5, 2.01, 2.01]
    staticObjectMsg.PositionVector = [0.0, 0.0, 0.0]
    staticObjectMsg.VelocityVector = [0., 0., 0.]
    staticObjectMsg.J20002Pfix = np.identity(3)
    stObMsg = messaging.SpicePlanetStateMsg().write(staticObjectMsg)


    scContact.AddSpiceBody("cube2.obj", stObMsg, 1.0, 1.0, 0.0)
    # scContact.AddOtherBody("surface.obj", stObMsg, 5.0, 1.0, 1.0)
    # scContact.externalBodies[0].states.r_BN_N = [[3.5], [2.0], [2.0]]  # m   - r_CN_N
    # scContact.externalBodies[0].states.v_BN_N = [[0.0], [0.0], [0.0]]  # m/s - v_CN_N
    # scContact.externalBodies[0].states.sigma_BN = [[0.0], [0.0], [0.0]]  # sigma_CN_B
    # scContact.externalBodies[0].states.omega_BN_B = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B
    # scContact.externalBodies[0].states.c_B = [[0.0], [0.0], [0.0]]
    # I2 = np.array([[((1. / 6.) * 2000. * 4.), 0., 0.],
    #       [0., ((1. / 6.) * 2000. * 4.), 0.],
    #       [0., 0, ((1. / 6.) * 2000. * 4.)]])
    # scContact.Bodies[1].states.m_SC = 2000.
    # scContact.Bodies[1].states.ISCPntB_B = unitTestSupport.np2EigenMatrix3d(I2)
    # scContact.Bodies[1].states.ISCPntB_B_inv = unitTestSupport.np2EigenMatrix3d(np.invert(I2))

    scObject.addDynamicEffector(scContact)
    scSim.AddModelToTask(simTaskName, scContact)


    #
    #   Setup data logging before the simulation is initialized
    #
    #numDataPoints = macros.sec2nano(1 / 60)
    samplingTime = macros.sec2nano(1 / 60) #simulationTime // (numDataPoints - 1)
    scStateRec = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, scStateRec)

    # scObject.hub.r_CN_NInit = [[1.51], [1.5], [1.5]]  # m   - r_CN_N
    # scObject.hub.v_CN_NInit = [[0.2], [0.0], [0.0]]  # m/s - v_CN_N
    # scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_CN_B
    # scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    # scObject.hub.r_CN_NInit = [[1.51], [1.7], [1.7]]  # m   - r_CN_N
    # scObject.hub.v_CN_NInit = [[0.2], [0.0], [0.0]]  # m/s - v_CN_N
    # scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_CN_B
    # scObject.hub.omega_BN_BInit = [[0.0], [0.1], [0.0]]  # rad/s - omega_CN_B

    # scObject.hub.r_CN_NInit = [[2.01], [1.7], [1.7]]  # m   - r_CN_N
    # scObject.hub.v_CN_NInit = [[3.5], [0.0], [0.0]]  # m/s - v_CN_N
    # scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.3]]  # sigma_CN_B
    # scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    # scObject.hub.r_CN_NInit = [[1.51], [1.7], [1.7]]  # m   - r_CN_N
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [-2.0]]  # m/s - v_CN_N
    # scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_CN_B
    # scObject.hub.omega_BN_BInit = [[0.1], [0.1], [0.1]]  # rad/s - omega_CN_B

    mrp = RigidBodyKinematics.C2MRP(RigidBodyKinematics.Mi(0.2, 2))
    scObject.hub.r_CN_NInit = [[0.0], [0.0], [1.5]]  # m   - r_CN_N
    # scObject.hub.v_CN_NInit = [[0.0], [0.0], [-0.7]]  # m/s - v_CN_N
    scObject.hub.sigma_BNInit = [[mrp[0]], [mrp[1]], [mrp[2]]]  # sigma_CN_B
    scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    # mrp = RigidBodyKinematics.C2MRP(RigidBodyKinematics.Mi(0.3, 2))
    # scObject.hub.r_CN_NInit = [[0.0], [1.0], [2.5]]  # m   - r_CN_N
    # scObject.hub.v_CN_NInit = [[0.0], [0.0], [-0.4]]  # m/s - v_CN_N
    # scObject.hub.sigma_BNInit = [[mrp[0]], [mrp[1]], [mrp[2]]]  # sigma_CN_B
    # scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    rNData1 = scStateRec.r_CN_N
    # rNData2 = scSim.pullMessageLogData(scObject2.scStateOutMsgName + '.r_CN_N', list(range(3)))
    vNData1 = scStateRec.v_CN_N
    # vNData2 = scSim.pullMessageLogData(scObject2.scStateOutMsgName + '.v_CN_N', list(range(3)))
    sigmaData1 = scStateRec.sigma_BN
    # sigmaData2 = scSim.pullMessageLogData(scObject2.scStateOutMsgName + '.sigma_BN', list(range(3)))
    omegaData1 = scStateRec.omega_BN_B
    # omegaData2 = scSim.pullMessageLogData(scObject2.scStateOutMsgName + '.omega_BN_B', list(range(3)))
    rNData2 = []
    vNData2 = []
    sigmaData2 = []
    omegaData2 = []
    mainPath = []
    otherPath = []
    for ii in range(rNData1.shape[0]):
        rNData2.append(staticObjectMsg.PositionVector)
        vNData2.append([0., 0., 0.])
        sigmaData2.append([0., 0., 0.])
        omegaData2.append([0., 0., 0.])

    rNData2 = np.asarray(rNData2)
    vNData2 = np.asarray(vNData2)
    sigmaData2 = np.asarray(sigmaData2)
    omegaData2 = np.asarray(omegaData2)

    for ii in range(0, rNData1.shape[0], 10):
        mainPath.append(np.array([rNData1[ii, 0], rNData1[ii, 1], rNData1[ii, 2],
                                  vNData1[ii, 0], vNData1[ii, 1], vNData1[ii, 2],
                                  sigmaData1[ii, 0], sigmaData1[ii, 1], sigmaData1[ii, 2],
                                  omegaData1[ii, 0], omegaData1[ii, 1], omegaData1[ii, 2]]))
        otherPath.append(np.array([rNData2[ii, 0], rNData2[ii, 1], rNData2[ii, 2],
                                  vNData2[ii, 0], vNData2[ii, 1], vNData2[ii, 2],
                                  sigmaData2[ii, 0], sigmaData2[ii, 1], sigmaData2[ii, 2],
                                  omegaData2[ii, 0], omegaData2[ii, 1], omegaData2[ii, 2]]))



    kineticData1 = []
    kineticData2 = []
    totalKinetic = []
    relVel = []

    for ii in range(rNData1.shape[0]):
        kineticData1.append(
            0.5 * scObject.hub.mHub * (np.linalg.norm(vNData1[ii, :]) ** 2) + 0.5 * omegaData1[ii, :]
                            @ scObject.hub.IHubPntBc_B @ omegaData1[ii, :] )
        totalKinetic.append( np.sqrt(kineticData1[ii]))
        relVel.append(np.linalg.norm(vNData1[ii, :]) - np.linalg.norm(vNData2[ii, :]))

    print(totalKinetic[-1] / totalKinetic[0])
    # surfaceGeo = GEO.SurfaceGeometry(["surface.obj"], [1.0])
    # landerGeo = GEO.LanderGeometry("Lander.obj", 1.0)
    # surfaceGeo = GEO.SurfaceGeometry(["cube2.obj"], [1.0])
    # landerGeo = GEO.LanderGeometry("cube2.obj", 1.0)


    fig1 = plt.figure(figsize=(5, 5))
    fig1.set_tight_layout(False)
    ax1 = p3.Axes3D(fig1, auto_add_to_figure=False)
    fig1.add_axes(ax1)

    ax1.set_xlim3d([-2.0, 2.0])
    ax1.set_xlabel('X')

    ax1.set_ylim3d([-2.0, 2.0])
    ax1.set_ylabel('Y')

    ax1.set_zlim3d([-1.0, 4.0])
    ax1.set_zlabel('Z')



    cube1 = []
    cube2 = []
    for ii in range(rNData1.shape[0]):
        cube1.append(create_vert_data(rNData1[ii, :], sigmaData1[ii, :]))
        cube2.append(create_vert_data(rNData2[ii, :], sigmaData2[ii, :]))

    #shapes = [ax1.add_collection3d(Poly3DCollection(cube1[0][ii])) for ii in range(12)]
    #shapes.extend([ax1.add_collection3d(Poly3DCollection(cube2[0][ii])) for ii in range(12)])

    shapes = [Poly3DCollection(cube1[0][ii]) for ii in range(12)]
    shapes.extend([Poly3DCollection(cube2[0][ii]) for ii in range(12)])

    for shape in shapes:
        ax1.add_collection3d(shape)


    polys = []
    for col in ax1.collections:
        polys.append(col)


    animat = animation.FuncAnimation(fig1, update_shapes, rNData1.shape[0], fargs=(cube1, cube2, polys),
                                     interval=60, blit=False)


    plt.show()

    










if __name__ == "__main__":
    run()