# ISC License
#
# Copyright (c) 2023, Laboratory for Atmospheric Space Physics, University of Colorado Boulder
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

import inspect

import matplotlib.pyplot as plt
import numpy as np
import os
import pickle
import pytest

open3d = pytest.importorskip("open3d")

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)

from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import scalingIterativeClosestPoint

def pythonSICP(pointCloud, referenceCloud, Np, scale_minMax = None):
    maxIterations = 10
    minError = 1E-10

    m = 3
    R_kmin1 = np.eye(m)
    s_kmin1 = 1
    if scale_minMax is not None:
        s_a_min = scale_minMax[0]
        s_b_max = scale_minMax[1]
    t_kmin1 = np.zeros(m)

    SRT = []
    for k in range(maxIterations):
        # correspondance
        correspondance_k = np.zeros(Np)
        q = np.zeros([Np, m])
        n = np.zeros([Np, m])
        m_ck = np.zeros([Np, m])

        for i in range(Np):
            correspondance_k[i] = np.argmin(np.linalg.norm(s_kmin1*np.dot(R_kmin1, pointCloud[i,:]) + t_kmin1 - referenceCloud, axis=1))
            m_ck[i, :] = referenceCloud[int(correspondance_k[i]), :]

        for i in range(Np):
            q[i,:] = pointCloud[i,:] - 1/Np*np.sum(pointCloud, axis=0)
            n[i,:] = m_ck[i,:] - 1/Np*np.sum(m_ck, axis=0)

        # Finding new S_k and R_k
        for j in range(maxIterations):
            H = np.zeros([m,m])
            for i in range(Np):
                H += s_kmin1 * np.outer(q[i,:], n[i,:])
            H /= Np
            U, Lambda, V = np.linalg.svd(H)
            if np.abs(np.linalg.det(np.dot(V.T, U.T)) - 1) < minError:
                R_k = np.dot(V.T, U.T)
            elif np.abs(np.linalg.det(np.dot(V.T, U.T)) + 1) < minError:
                zeroIndices = []
                for i in range(len(Lambda)):
                    if np.abs(Lambda[i]) < minError:
                        zeroIndices.append(i)
                if len(zeroIndices) == 1:
                    I_reflection = np.eye(len(Lambda))
                    I_reflection[zeroIndices[0], zeroIndices[0]] = -1
                    R_k = np.dot(np.dot(V.T, I_reflection), U.T)
                else:
                    R_k = R_kmin1

            s_kn = 1
            sumNumerator = 0
            sumDenomerator = 0
            for i in range(Np):
                sumNumerator += np.dot(np.dot(n[i,:],R_k), q[i,:])
                sumDenomerator += np.dot(q[i,:], q[i,:])
            s_kn = sumNumerator/sumDenomerator
            if scale_minMax is not None:
                if s_kn < s_kmin1:
                    s_kn = s_a_min
                if s_kn > s_b_max:
                    s_kn = s_b_max
            if j > 0:
                if np.linalg.norm(s_kn - s_knmin1) < minError:
                    s_k = s_kn
                    break
            s_knmin1 = s_kn

        tkSum1 = 1/Np*np.sum(m_ck, axis=0)
        tkSum2 = 0
        for i in range(Np):
            tkSum2 += s_k*np.dot(R_k, pointCloud[i,:])
        tk = tkSum1 - tkSum2/Np

        if np.linalg.norm(s_k - s_kmin1) < minError and np.linalg.norm(R_k - R_kmin1) < minError and np.linalg.norm(tk - t_kmin1) < minError:
            break
        SRT.append([s_k, R_k, tk])

        R_kmin1 = R_k
        s_kmin1 = s_k
        t_kmin1 = tk

    return s_k, R_k, tk, SRT

def convertToPickle(path):
    from plyfile import PlyData, PlyElement
    with open(path, 'rb') as f:
        plydata = PlyData.read(f)

    reference = np.zeros([len(plydata.elements[0]), 3])
    for i in range(len(plydata.elements[0])):
        reference[i, 0] = plydata.elements[0][i][0]
        reference[i, 1] = plydata.elements[0][i][1]
        reference[i, 2] = plydata.elements[0][i][2]

    file1 = open(path[:-4] + ".pickle", 'wb')
    pickle.dump(reference, file1)
    file1.close()

def saveCostFunctions(costFunction, test):
    plt.figure(1)
    plt.plot(costFunction,
             color='k',
             )
    plt.xlabel('Iterations [-]')
    plt.ylabel(r'Cost Function for test ' + test)
    plt.title(test)
    plt.show()

def saveImages(data, reference, module_Ss, module_Rs, module_Ts):
    vis = open3d.visualization.Visualizer()
    vis.create_window()

    # geometry is the point cloud used in the animation
    geometry = open3d.geometry.PointCloud()
    static = open3d.geometry.PointCloud()
    static.points = open3d.utility.Vector3dVector(reference)
    vis.add_geometry(static)
    vis.add_geometry(geometry)

    for i in range(len(module_Ss)):
        pointsTransformed = np.zeros(np.shape(data))
        for j in range(len(data[:,0])):
            pointsTransformed[j,:] = module_Ss[i]*np.dot(module_Rs[i,:,:], data[j,:]) + module_Ts[i,:]
        geometry.points = open3d.utility.Vector3dVector(pointsTransformed)
        vis.update_geometry(geometry)
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image("temp_%04d.jpg" % i)

    vis.run()

def runModule(data, reference, numberPoints, iters = 100):
    unitTaskName = "unitTask"
    unitProcessName = "TestProcess"
    unitTestSim = SimulationBaseClass.SimBaseClass()

    testProcessRate = macros.sec2nano(0.5)
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(unitTestSim.CreateNewTask(unitTaskName, testProcessRate))
    sicp = scalingIterativeClosestPoint.ScalingIterativeClosestPoint()
    sicp.maxIterations = iters
    sicp.numberScalePoints = 100
    sicp.errorTolerance = 1E-10
    sicp.scalingMax = 1.1
    sicp.scalingMin = 0.9
    unitTestSim.AddModelToTask(unitTaskName, sicp, sicp)

    # Create the input messages.
    inputPointCloud = messaging.PointCloudMsgPayload()
    referencePointCloud = messaging.PointCloudMsgPayload()
    icpInitialCondition = messaging.SICPMsgPayload()

    inputPointCloud.points = data.flatten().tolist()
    inputPointCloud.numberOfPoints = numberPoints
    inputPointCloud.timeTag = 1
    inputPointCloud.valid = True
    inputPointCloudMsg = messaging.PointCloudMsg().write(inputPointCloud)
    sicp.measuredPointCloud.subscribeTo(inputPointCloudMsg)

    icpInitialCondition.valid = False
    initialConditionMsg = messaging.SICPMsg().write(icpInitialCondition)
    sicp.initialCondition.subscribeTo(initialConditionMsg)

    referencePointCloud.points = reference.flatten().tolist()
    referencePointCloud.numberOfPoints = numberPoints
    referencePointCloud.timeTag = 0
    referencePointCloud.valid = True
    referencePointCloudMsg = messaging.PointCloudMsg().write(referencePointCloud)
    sicp.referencePointCloud.subscribeTo(referencePointCloudMsg)

    dataSICPLog = sicp.outputSICPData.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataSICPLog)
    outputLog = sicp.outputPointCloud.recorder()
    unitTestSim.AddModelToTask(unitTaskName, outputLog)

    unitTestSim.InitializeSimulation()
    unitTestSim.ConfigureStopTime(testProcessRate)
    unitTestSim.ExecuteSimulation()

    outputIterations = dataSICPLog.numberOfIteration[-1]
    outputSs = dataSICPLog.scaleFactor[-1][:outputIterations]
    outputRs = dataSICPLog.rotationMatrix[-1][:9*outputIterations]
    outputTs = dataSICPLog.translation[-1][:3*outputIterations]

    outputCloudValid = outputLog.valid[-1]
    outputCloudTimetag = outputLog.timeTag[-1]
    outputCloudNumberPoints = outputLog.numberOfPoints[-1]
    outputCloud = outputLog.points[-1][:3*outputCloudNumberPoints]

    module_Rs = np.zeros([outputIterations, 3, 3])
    module_Ss = np.zeros([outputIterations])
    module_Ts = np.zeros([outputIterations, 3])
    module_cloud = np.zeros([outputCloudNumberPoints, 3])
    for i in range(outputIterations):
        module_Rs[i, :, :] = outputRs[i*9:(i +1)*9].reshape([3,3])
        module_Ss[i] = outputSs[i]
        module_Ts[i, :] = outputTs[i*3:(i+1)*3]
    for i in range(outputCloudNumberPoints):
        module_cloud[i, 0] = outputCloud[i]
        module_cloud[i, 1] = outputCloud[i + outputCloudNumberPoints]
        module_cloud[i, 2] = outputCloud[i + 2*outputCloudNumberPoints]

    return module_Rs, module_Ts, module_Ss, module_cloud
