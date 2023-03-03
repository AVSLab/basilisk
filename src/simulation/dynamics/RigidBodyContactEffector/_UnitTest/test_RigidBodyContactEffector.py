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
import pandas as pd
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
    simulationTime = macros.sec2nano(1.0)

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

    scObject.hub.mHub = 5.0  # kg - spacecraft mass
    I = [((1./6.)*scObject.hub.mHub*4.), 0., 0.,
         0., ((1./6.)*scObject.hub.mHub*4.), 0.,
         0., 0., ((1./6.)*scObject.hub.mHub*4.)]

    scObject.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    scSim.AddModelToTask(simTaskName, scObject, None, 10)

    scObject2 = spacecraft.Spacecraft()
    scObject2.ModelTag = "Secondary"
    integratorObject2 = svIntegrators.svIntegratorRKF45(scObject2)
    scObject2.setIntegrator(integratorObject2)

    scObject2.hub.mHub = 5.0  # kg - spacecraft mass
    I = [((1. / 6.) * scObject2.hub.mHub * 4.), 0., 0.,
         0., ((1. / 6.) * scObject2.hub.mHub * 4.), 0.,
         0., 0., ((1. / 6.) * scObject2.hub.mHub * 4.)]

    scObject2.hub.IHubPntBc_B = unitTestSupport.np2EigenMatrix3d(I)

    scSim.AddModelToTask(simTaskName, scObject2, None, 1)


    scContact = RigidBodyContactEffector.RigidBodyContactEffector()
    scContact.maxPosError = 1e-4
    scContact.simTimeStep = 0.01
    # scContact.slipTolerance = 1e-6 # not currently used
    scContact.collisionIntegrationStep = 1e-5
    scContact.maxBoundingBoxDim = 0.5
    scContact.minBoundingBoxDim = 0.01
    scContact.boundingBoxFF = 1.2 # "fudge factor" (a multiplied value)
    scContact.LoadSpacecraftBody("highrescube.obj", scObject.ModelTag, scObject.scStateOutMsg, scObject.scMassOutMsg, 1.0, 1.0, 0.0)
    scContact.LoadSpacecraftBody("highrescube.obj", scObject2.ModelTag, scObject2.scStateOutMsg, scObject2.scMassOutMsg,
                                 1.0, 1.0, 0.0)
    # scContact.mainBody.modelTag = scObject.ModelTag
    # scContact.mainBody.boundingRadius = 3.5
    # scContact.mainBody.boundingRadius = 1.5


    # staticObjectMsg = messaging.SpicePlanetStateMsgPayload()
    # # staticObjectMsg.PositionVector = [3.5, 2.01, 2.01]
    # staticObjectMsg.PositionVector = [0.0, 0.0, 0.0]
    # staticObjectMsg.VelocityVector = [0., 0., 0.]
    # staticObjectMsg.J20002Pfix = np.identity(3) #RigidBodyKinematics.Mi(np.deg2rad(90),3)
    # stObMsg = messaging.SpicePlanetStateMsg().write(staticObjectMsg)
    # scContact.AddSpiceBody("highrescube.obj", stObMsg, 1.0, 1.0, 0.0)


    scObject.addDynamicEffector(scContact)
    scObject2.addDynamicEffector(scContact)
    scSim.AddModelToTask(simTaskName, scContact)

    scObject2.hub.r_CN_NInit = [[0.0], [0.0], [0.0]]  # m   - r_CN_N
    scObject2.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]  # m/s - v_CN_N
    scObject2.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_CN_B
    scObject2.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    #
    #   Setup data logging before the simulation is initialized
    #
    #numDataPoints = macros.sec2nano(1 / 60)
    samplingTime = macros.sec2nano(1 / 60) #simulationTime // (numDataPoints - 1)
    scStateRec = scObject.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, scStateRec)

    scStateRec2 = scObject2.scStateOutMsg.recorder()
    scSim.AddModelToTask(simTaskName, scStateRec2)

    # these are trying out different initial states, for a complete unit test would want to try out different combinations of face on face, edge on edge, vertex to face etc.
    #   also try out different restitutions, frictions, etc.

    # scObject.hub.r_CN_NInit = [[1.51], [1.5], [1.5]]  # m   - r_CN_N
    # scObject.hub.v_CN_NInit = [[0.2], [0.0], [0.0]]  # m/s - v_CN_N
    # scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_CN_B
    # scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    # scObject.hub.r_CN_NInit = [[1.51], [1.7], [1.7]]  # m   - r_CN_N
    # scObject.hub.v_CN_NInit = [[0.2], [0.0], [0.0]]  # m/s - v_CN_N
    # scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_CN_B
    # scObject.hub.omega_BN_BInit = [[0.0], [0.1], [0.0]]  # rad/s - omega_CN_B

    # scObject.hub.r_CN_NInit = [[2.01], [1.7], [1.7]]  # m   - r_CN_N
    # scObject.hub.v_CN_NInit = [[0.0], [0.0], [0.0]]  # m/s - v_CN_N
    # scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.3]]  # sigma_CN_B
    # scObject.hub.omega_BN_BInit = [[0.0], [0.0], [0.0]]  # rad/s - omega_CN_B

    # scObject.hub.r_CN_NInit = [[0.0], [0.0], [1.05]]  # m   - r_CN_N
    scObject.hub.v_CN_NInit = [[0.0], [0.0], [-2.0]]  # m/s - v_CN_N
    # scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]  # sigma_CN_B
    # scObject.hub.omega_BN_BInit = [[0.0], [1.0], [0.0]]  # rad/s - omega_CN_B

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
    vNData1 = scStateRec.v_CN_N
    sigmaData1 = scStateRec.sigma_BN
    omegaData1 = scStateRec.omega_BN_B

    rNData2 = scStateRec2.r_CN_N
    vNData2 = scStateRec2.v_CN_N
    sigmaData2 = scStateRec2.sigma_BN
    omegaData2 = scStateRec2.omega_BN_B

    data = {
        "rNData1 [m]": rNData1,
        "vNData1 [m/s]": vNData1,
        "sigmaData1": sigmaData1,
        "omegaData1 [rad/s]": omegaData1,
        "rNData2 [m]": rNData2,
        "vNData2 [m/s]": vNData2,
        "sigmaData2": sigmaData2,
        "omegaData2 [rad/s]": omegaData2,
    }

    dfData = pd.DataFrame()
    for var in data:
        for axis in range(data[var].shape[1]):
            col = pd.DataFrame({var: data[var][:,axis]})
            dfData = pd.concat([dfData,col],axis=1)

    dfData.to_csv('scData.csv')

    # rNData2 = []
    # vNData2 = []
    # sigmaData2 = []
    # omegaData2 = []
    mainPath = []
    otherPath = []
    # for ii in range(rNData1.shape[0]):
    #     rNData2.append(staticObjectMsg.PositionVector)
    #     vNData2.append([0., 0., 0.])
    #     sigmaData2.append([0., 0., 0.])
    #     omegaData2.append([0., 0., 0.])

    # rNData2 = np.asarray(rNData2)
    # vNData2 = np.asarray(vNData2)
    # sigmaData2 = np.asarray(sigmaData2)
    # omegaData2 = np.asarray(omegaData2)

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

        kineticData2.append(
            0.5 * scObject2.hub.mHub * (np.linalg.norm(vNData2[ii, :]) ** 2) + 0.5 * omegaData2[ii, :]
            @ scObject2.hub.IHubPntBc_B @ omegaData2[ii, :])

        totalKinetic.append( np.sqrt(kineticData1[ii] + kineticData2[ii]))
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