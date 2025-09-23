#
#   Unit Test Script
#   Module Name:        pixelLineConverter.py
#   Creation Date:      May 16, 2019
#   Author:             Thibaud Teil
#

import inspect
import os

import numpy as np
from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import horizonOpNav
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, unitTestSupport, macros

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))


def back_substitution(A, b):
    n = b.size
    x = np.zeros_like(b)

    if A[-1, -1] == 0:
        raise ValueError

    x[-1] = b[-1] / A[-1, -1]
    for i in range(n - 2, -1, -1):
        sum = 0
        for j in range(i, n):
            sum += A[i, j] * x[j]
        x[i] = (b[i] - sum) / A[i, i]
    return x


def test_horizonOpNav():
    """
    Unit test for Horizon Navigation. The unit test specifically covers:

        1. Individual methods: This module contains a back substitution method as well as a QR decomposition.
            This test ensures that they are working properly with a direct test of the method input/outputs with
            expected results

        2. State and Covariances: This unit test also computes the state estimate and covariance in python. This is
            compared directly to the output from the module for exact matching.

    The Horizon Nav module gives the spacecraft position given a limb input. This test ensures that the results are as
    expected both for the state estimate and the covariance associated with the measurement.
    """
    [testResults, testMessage] = horizonOpNav_methods()
    assert testResults < 1, testMessage
    [testResults, testMessage] = horizonOpNav_update()
    assert testResults < 1, testMessage


def horizonOpNav_methods():
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    ###################################################################################
    ## Testing QR decomp
    ###################################################################################
    Hinput = np.array(
        [[1, 2, 3], [1, 20, 3], [3, 0, 1], [2, 1, 0], [20, -1, -5], [0, 10, -5]]
    )
    numStates = np.shape(Hinput)[0]
    # Fill in the variables for the test
    Qin = horizonOpNav.new_doubleArray(3 * numStates)
    Rin = horizonOpNav.new_doubleArray(3 * 3)
    Hin = horizonOpNav.new_doubleArray(numStates * 3)
    for j in range(numStates * 3):
        horizonOpNav.doubleArray_setitem(Qin, j, 0)
    for j in range(3 * 3):
        horizonOpNav.doubleArray_setitem(Rin, j, 0)
    for j in range(numStates * 3):
        horizonOpNav.doubleArray_setitem(Hin, j, Hinput.flatten().tolist()[j])
    horizonOpNav.QRDecomp(Hin, numStates, Qin, Rin)

    Qout = []
    for j in range(3 * numStates):
        Qout.append(horizonOpNav.doubleArray_getitem(Qin, j))
    Rout = []
    for j in range(3 * 3):
        Rout.append(horizonOpNav.doubleArray_getitem(Rin, j))

    q, r = np.linalg.qr(Hinput)

    Rpy = np.zeros([3, 3])
    Qpy = np.zeros([numStates, 3])
    for i in range(0, 3):
        Qpy[:, i] = Hinput[:, i]
        for j in range(i):
            Rpy[j, i] = np.dot(Qpy[:, j], Hinput[:, i])
            Qpy[:, i] = Qpy[:, i] - Rpy[j, i] * Qpy[:, j]
        Rpy[i, i] = np.linalg.norm(Qpy[:, i])
        Qpy[:, i] = 1 / Rpy[i, i] * Qpy[:, i]

    Qtest = np.array(Qout).reshape([numStates, 3])
    Rtest = np.array(Rout).reshape(3, 3)
    errorNorm1 = np.linalg.norm(Qpy - Qtest)
    errorNorm2 = np.linalg.norm(Rpy - Rtest)
    if errorNorm1 > 1.0e-10:
        print(errorNorm1, "QR decomp")
        testFailCount += 1
        testMessages.append("QR decomp Failure in Q" + "\n")
    if errorNorm2 > 1.0e-10:
        print(errorNorm2, "QR decomp")
        testFailCount += 1
        testMessages.append("QR decomp Failure in R" + "\n")
    errorNorm1 = np.linalg.norm(q + Qtest)
    errorNorm2 = np.linalg.norm(r[:3, :3] + Rtest)
    if errorNorm1 > 1.0e-10:
        print(errorNorm1, "QR decomp")
        testFailCount += 1
        testMessages.append("QR decomp Failure in Q" + "\n")
    if errorNorm2 > 1.0e-10:
        print(errorNorm2, "QR decomp")
        testFailCount += 1
        testMessages.append("QR decomp Failure in R" + "\n")

    ###################################################################################
    ## Testing Back Sub
    ###################################################################################
    V = np.ones(3)
    nIn = horizonOpNav.new_doubleArray(3)
    VIn = horizonOpNav.new_doubleArray(3)
    RIn = horizonOpNav.new_doubleArray(numStates * 3)
    for i in range(3):
        horizonOpNav.doubleArray_setitem(nIn, i, 0.0)
    for i in range(3 * 3):
        horizonOpNav.doubleArray_setitem(RIn, i, r.flatten().tolist()[i])
    for i in range(3):
        horizonOpNav.doubleArray_setitem(VIn, i, V.flatten().tolist()[i])

    horizonOpNav.BackSub(RIn, VIn, 3, nIn)
    BackSubOut = []
    for i in range(3):
        BackSubOut.append(horizonOpNav.doubleArray_getitem(nIn, i))

    exp = back_substitution(r[:3, :3], V)

    BackSubOut = np.array(BackSubOut)
    errorNorm = np.linalg.norm(exp - BackSubOut)
    if errorNorm > 1.0e-10:
        print(errorNorm, "BackSub")
        testFailCount += 1
        testMessages.append("BackSub Failure " + "\n")

    return [testFailCount, "".join(testMessages)]


###################################################################################
## Testing dynamics matrix computation
###################################################################################
def horizonOpNav_update():
    # Create a sim module as an empty container
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)
    unitTestSim = SimulationBaseClass.SimBaseClass()

    # Create test thread
    testProcessRate = macros.sec2nano(0.5)  # update process rate update time
    testProc = unitTestSim.CreateNewProcess(unitProcessName)
    testProc.addTask(
        unitTestSim.CreateNewTask(unitTaskName, testProcessRate)
    )  # Add a new task to the process

    # Construct the ephemNavConverter module
    # Set the names for the input messages
    opNav = horizonOpNav.horizonOpNav()
    opNav.noiseSF = 2
    # ephemNavConfig.outputState = simFswInterfaceMessages.NavTransIntMsg()

    # This calls the algContain to setup the selfInit, update, and reset
    opNav.ModelTag = "limbNav"

    # Add the module to the task
    unitTestSim.AddModelToTask(unitTaskName, opNav)

    # These are example points for fitting used from an image processing algorithm
    inputPoints = [
        226.0,
        113.0,
        227.0,
        113.0,
        223.0,
        114.0,
        224.0,
        114.0,
        225.0,
        114.0,
        219.0,
        115.0,
        220.0,
        115.0,
        221.0,
        115.0,
        222.0,
        115.0,
        215.0,
        116.0,
        216.0,
        116.0,
        217.0,
        116.0,
        218.0,
        116.0,
        212.0,
        117.0,
        213.0,
        117.0,
        214.0,
        117.0,
        209.0,
        118.0,
        210.0,
        118.0,
        211.0,
        118.0,
        205.0,
        119.0,
        206.0,
        119.0,
        207.0,
        119.0,
        208.0,
        119.0,
        204.0,
        120.0,
        205.0,
        120.0,
        201.0,
        121.0,
        202.0,
        121.0,
        203.0,
        121.0,
        199.0,
        122.0,
        200.0,
        122.0,
        197.0,
        123.0,
        198.0,
        123.0,
        195.0,
        124.0,
        196.0,
        124.0,
        193.0,
        125.0,
        194.0,
        125.0,
        191.0,
        126.0,
        192.0,
        126.0,
        189.0,
        127.0,
        190.0,
        127.0,
        187.0,
        128.0,
        188.0,
        128.0,
        185.0,
        129.0,
        186.0,
        129.0,
        183.0,
        130.0,
        184.0,
        130.0,
        181.0,
        131.0,
        182.0,
        131.0,
        180.0,
        132.0,
        181.0,
        132.0,
        178.0,
        133.0,
        179.0,
        133.0,
        177.0,
        134.0,
        178.0,
        134.0,
        175.0,
        135.0,
        176.0,
        135.0,
        174.0,
        136.0,
        175.0,
        136.0,
        172.0,
        137.0,
        173.0,
        137.0,
        171.0,
        138.0,
        172.0,
        138.0,
        170.0,
        139.0,
        171.0,
        139.0,
        168.0,
        140.0,
        169.0,
        140.0,
        167.0,
        141.0,
        168.0,
        141.0,
        166.0,
        142.0,
        167.0,
        142.0,
        164.0,
        143.0,
        165.0,
        143.0,
        163.0,
        144.0,
        164.0,
        144.0,
        162.0,
        145.0,
        163.0,
        145.0,
        161.0,
        146.0,
        162.0,
        146.0,
        160.0,
        147.0,
        161.0,
        147.0,
        159.0,
        148.0,
        160.0,
        148.0,
        158.0,
        149.0,
        159.0,
        149.0,
        156.0,
        150.0,
        157.0,
        150.0,
        155.0,
        151.0,
        156.0,
        151.0,
        154.0,
        152.0,
        155.0,
        152.0,
        153.0,
        153.0,
        154.0,
        153.0,
        153.0,
        154.0,
        152.0,
        155.0,
        151.0,
        156.0,
        152.0,
        156.0,
        150.0,
        157.0,
        151.0,
        157.0,
        149.0,
        158.0,
        150.0,
        158.0,
        148.0,
        159.0,
        149.0,
        159.0,
        147.0,
        160.0,
        148.0,
        160.0,
        146.0,
        161.0,
        147.0,
        161.0,
        145.0,
        162.0,
        146.0,
        162.0,
        145.0,
        163.0,
        144.0,
        164.0,
        143.0,
        165.0,
        144.0,
        165.0,
        142.0,
        166.0,
        143.0,
        166.0,
        142.0,
        167.0,
        141.0,
        168.0,
        140.0,
        169.0,
        141.0,
        169.0,
        139.0,
        170.0,
        140.0,
        170.0,
        139.0,
        171.0,
        138.0,
        172.0,
        137.0,
        173.0,
        138.0,
        173.0,
        137.0,
        174.0,
        136.0,
        175.0,
        135.0,
        176.0,
        136.0,
        176.0,
        135.0,
        177.0,
        134.0,
        178.0,
        133.0,
        179.0,
        134.0,
        179.0,
        133.0,
        180.0,
        132.0,
        181.0,
        132.0,
        182.0,
        131.0,
        183.0,
        131.0,
        184.0,
        130.0,
        185.0,
        129.0,
        186.0,
        130.0,
        186.0,
        129.0,
        187.0,
        128.0,
        188.0,
        128.0,
        189.0,
        127.0,
        190.0,
        127.0,
        191.0,
        126.0,
        192.0,
        126.0,
        193.0,
        125.0,
        194.0,
        125.0,
        195.0,
        125.0,
        196.0,
        124.0,
        197.0,
        124.0,
        198.0,
        123.0,
        199.0,
        123.0,
        200.0,
        122.0,
        201.0,
        122.0,
        202.0,
        122.0,
        203.0,
        121.0,
        204.0,
        120.0,
        205.0,
        121.0,
        205.0,
        120.0,
        206.0,
        120.0,
        207.0,
        120.0,
        208.0,
        119.0,
        209.0,
        119.0,
        210.0,
        119.0,
        211.0,
        118.0,
        212.0,
        118.0,
        213.0,
        118.0,
        214.0,
        117.0,
        215.0,
        117.0,
        216.0,
        117.0,
        217.0,
        117.0,
        218.0,
        116.0,
        219.0,
        116.0,
        220.0,
        116.0,
        221.0,
        116.0,
        222.0,
        115.0,
        223.0,
        115.0,
        224.0,
        115.0,
        225.0,
        115.0,
        226.0,
        114.0,
        227.0,
        114.0,
        228.0,
        114.0,
        229.0,
        114.0,
        230.0,
        114.0,
        231.0,
        114.0,
        232.0,
        113.0,
        233.0,
        113.0,
        234.0,
        113.0,
        235.0,
        113.0,
        236.0,
        113.0,
        237.0,
        113.0,
        238.0,
        113.0,
        239.0,
        112.0,
        240.0,
        112.0,
        241.0,
        112.0,
        242.0,
        112.0,
        243.0,
        112.0,
        244.0,
        112.0,
        245.0,
        112.0,
        246.0,
        112.0,
        247.0,
        112.0,
        248.0,
        112.0,
        249.0,
        112.0,
        250.0,
        112.0,
        251.0,
        112.0,
        252.0,
        112.0,
        253.0,
        112.0,
        254.0,
        111.0,
        255.0,
        111.0,
        256.0,
        112.0,
        257.0,
        112.0,
        258.0,
        112.0,
        259.0,
        112.0,
        260.0,
        112.0,
        261.0,
        112.0,
        262.0,
        112.0,
        263.0,
        112.0,
        264.0,
        112.0,
        265.0,
        112.0,
        266.0,
        112.0,
        267.0,
        112.0,
        268.0,
        112.0,
        269.0,
        112.0,
        270.0,
        112.0,
        271.0,
        113.0,
        272.0,
        113.0,
        273.0,
        113.0,
        274.0,
        113.0,
        275.0,
        113.0,
        276.0,
        113.0,
        277.0,
        113.0,
        278.0,
        114.0,
        279.0,
        114.0,
        280.0,
        114.0,
        281.0,
        114.0,
        282.0,
        114.0,
        283.0,
        114.0,
        284.0,
        115.0,
        285.0,
        115.0,
        286.0,
        115.0,
        287.0,
        115.0,
        288.0,
        116.0,
        289.0,
        116.0,
        290.0,
        116.0,
        291.0,
        116.0,
        292.0,
        117.0,
        293.0,
        117.0,
        294.0,
        117.0,
        295.0,
        117.0,
        296.0,
        118.0,
        297.0,
        118.0,
        298.0,
        118.0,
        299.0,
        119.0,
        300.0,
        119.0,
        301.0,
        119.0,
        302.0,
        120.0,
        303.0,
        120.0,
        304.0,
        120.0,
        305.0,
        121.0,
        306.0,
        121.0,
        307.0,
        122.0,
        308.0,
        122.0,
        309.0,
        122.0,
        310.0,
        123.0,
        311.0,
        123.0,
        312.0,
        124.0,
        313.0,
        124.0,
        314.0,
        125.0,
        315.0,
        125.0,
        316.0,
        125.0,
        317.0,
        126.0,
        318.0,
        126.0,
        319.0,
        127.0,
        320.0,
        127.0,
        321.0,
        128.0,
        322.0,
        128.0,
        323.0,
        129.0,
        324.0,
        129.0,
        325.0,
        130.0,
        325.0,
        130.0,
        326.0,
        131.0,
        327.0,
        131.0,
        328.0,
        132.0,
        329.0,
        132.0,
        330.0,
        133.0,
        331.0,
        133.0,
        332.0,
        134.0,
        332.0,
        134.0,
        333.0,
        135.0,
        334.0,
        135.0,
        335.0,
        136.0,
        335.0,
        136.0,
        336.0,
        137.0,
        337.0,
        137.0,
        338.0,
        138.0,
        338.0,
        138.0,
        339.0,
        139.0,
        340.0,
        139.0,
        341.0,
        140.0,
        341.0,
        140.0,
        342.0,
        141.0,
        342.0,
        141.0,
        343.0,
        142.0,
        344.0,
        142.0,
        345.0,
        143.0,
        345.0,
        143.0,
        346.0,
        144.0,
        346.0,
        144.0,
        347.0,
        145.0,
        348.0,
        145.0,
        349.0,
        146.0,
        349.0,
        146.0,
        350.0,
        147.0,
        350.0,
        147.0,
        351.0,
        148.0,
        351.0,
        148.0,
        352.0,
        149.0,
        352.0,
        149.0,
        353.0,
        150.0,
        353.0,
        150.0,
        354.0,
        151.0,
        354.0,
        151.0,
        355.0,
        152.0,
        356.0,
        152.0,
        357.0,
        153.0,
        357.0,
        153.0,
        358.0,
        154.0,
        358.0,
        154.0,
        359.0,
        155.0,
        359.0,
        155.0,
        360.0,
        156.0,
        360.0,
        156.0,
        361.0,
        157.0,
        361.0,
        158.0,
        362.0,
        159.0,
        362.0,
        159.0,
        363.0,
        160.0,
        363.0,
        160.0,
        364.0,
        161.0,
        364.0,
        161.0,
        365.0,
        162.0,
        365.0,
        162.0,
        366.0,
        163.0,
        366.0,
        163.0,
        367.0,
        164.0,
        367.0,
        164.0,
        368.0,
        165.0,
        368.0,
        166.0,
        369.0,
        167.0,
        369.0,
        167.0,
        370.0,
        168.0,
        370.0,
        168.0,
        371.0,
        169.0,
        371.0,
        169.0,
        372.0,
        170.0,
        372.0,
        171.0,
        373.0,
        172.0,
        373.0,
        172.0,
        374.0,
        173.0,
        374.0,
        174.0,
        375.0,
        175.0,
        375.0,
        175.0,
        376.0,
        176.0,
        376.0,
        177.0,
        377.0,
        178.0,
        377.0,
        178.0,
        378.0,
        179.0,
        378.0,
        180.0,
        379.0,
        181.0,
        379.0,
        181.0,
        380.0,
        182.0,
        380.0,
        183.0,
        381.0,
        184.0,
        381.0,
        185.0,
        382.0,
        186.0,
        382.0,
        187.0,
        383.0,
        188.0,
        383.0,
        188.0,
        384.0,
        189.0,
        384.0,
        190.0,
        385.0,
        191.0,
        385.0,
        192.0,
        386.0,
    ]

    # Create the input messages.
    inputCamera = messaging.CameraConfigMsgPayload()
    inputLimbMsg = messaging.OpNavLimbMsgPayload()
    inputAtt = messaging.NavAttMsgPayload()

    # Set camera
    inputCamera.fieldOfView = 2.0 * np.arctan(
        10 * 1e-3 / 2.0 / 1.0
    )  # 2*arctan(s/2 / f)
    inputCamera.resolution = [512, 512]
    inputCamera.sigma_CB = [1.0, 0.2, 0.3]
    camInMsg = messaging.CameraConfigMsg().write(inputCamera)
    opNav.cameraConfigInMsg.subscribeTo(camInMsg)

    # Set circles
    inputLimbMsg.valid = 1
    inputLimbMsg.limbPoints = inputPoints
    inputLimbMsg.numLimbPoints = int(len(inputPoints) / 2)
    inputLimbMsg.timeTag = 12345
    limbInMsg = messaging.OpNavLimbMsg().write(inputLimbMsg)
    opNav.limbInMsg.subscribeTo(limbInMsg)

    # Set attitude
    inputAtt.sigma_BN = [0.6, 1.0, 0.1]
    attInMsg = messaging.NavAttMsg().write(inputAtt)
    opNav.attInMsg.subscribeTo(attInMsg)

    # Set module for Mars
    opNav.planetTarget = 2
    dataLog = opNav.opNavOutMsg.recorder()
    unitTestSim.AddModelToTask(unitTaskName, dataLog)

    # Initialize the simulation
    unitTestSim.InitializeSimulation()
    # The result isn't going to change with more time. The module will continue to produce the same result
    unitTestSim.ConfigureStopTime(testProcessRate)  # seconds to stop simulation
    unitTestSim.ExecuteSimulation()

    # Truth Vlaues
    ############################
    Q = np.eye(3)
    B = np.zeros([3, 3])
    Q *= 1 / (3396.19 * 1e3)  # km
    # Q[2,2] = 1/(3376.2*1E3)

    numPoints = int(len(inputPoints) / 2)

    CB = rbk.MRP2C(inputCamera.sigma_CB)
    BN = rbk.MRP2C(inputAtt.sigma_BN)
    CN = np.dot(CB, BN)
    B = np.dot(Q, CN.T)

    # Transf camera to meters
    alpha = 0
    up = inputCamera.resolution[0] / 2
    vp = inputCamera.resolution[1] / 2
    pX = 2.0 * np.tan(
        inputCamera.fieldOfView
        * inputCamera.resolution[0]
        / inputCamera.resolution[1]
        / 2.0
    )
    pY = 2.0 * np.tan(inputCamera.fieldOfView / 2.0)
    d_x = inputCamera.resolution[0] / pX
    d_y = inputCamera.resolution[1] / pY

    transf = np.zeros([3, 3])
    transf[0, 0] = 1 / d_x
    transf[1, 1] = 1 / d_y
    transf[2, 2] = 1
    transf[0, 1] = -alpha / (d_x * d_y)
    transf[0, 2] = (alpha * vp - d_y * up) / (d_x * d_y)
    transf[1, 2] = -vp / (d_y)

    s = np.zeros([numPoints, 3])
    sBar = np.zeros([numPoints, 3])
    sBarPrime = np.zeros([numPoints, 3])
    H = np.zeros([numPoints, 3])
    for i in range(numPoints):
        s[i, :] = np.dot(
            transf, np.array([inputPoints[2 * i], inputPoints[2 * i + 1], 1])
        )
        sBar[i, :] = np.dot(B, s[i, :])
        sBarPrime[i, :] = sBar[i, :] / np.linalg.norm(sBar[i, :])
        H[i, :] = sBarPrime[i, :]

    # QR H
    Rpy = np.zeros([3, 3])
    Qpy = np.zeros([numPoints, 3])
    for i in range(0, 3):
        Qpy[:, i] = H[:, i]
        for j in range(i):
            Rpy[j, i] = np.dot(Qpy[:, j], H[:, i])
            Qpy[:, i] = Qpy[:, i] - Rpy[j, i] * Qpy[:, j]
        Rpy[i, i] = np.linalg.norm(Qpy[:, i])
        Qpy[:, i] = 1 / Rpy[i, i] * Qpy[:, i]

    errorNorm1 = np.linalg.norm(np.dot(Qpy, Rpy) - H)
    if errorNorm1 > 1.0e-8:
        print(errorNorm1, "QR decomp")
        testFailCount += 1
        testMessages.append("QR decomp Failure in update test " + "\n")

    # Back Sub
    RHS = np.dot(Qpy.T, np.ones(numPoints))
    n = back_substitution(Rpy, RHS)
    n_test = np.dot(np.linalg.inv(Rpy), RHS)

    R_s = (
        (opNav.noiseSF * inputCamera.resolution[0] / (numPoints)) ** 2
        / d_x**2
        * np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]])
    )
    R_s = np.dot(np.dot(B, R_s), B.T)
    R_yInv = np.zeros([numPoints, numPoints])
    for i in range(numPoints):
        J = (
            1.0
            / np.linalg.norm(sBar[i, :])
            * np.dot(n, np.eye(3) - np.outer(sBarPrime[i, :], sBarPrime[i, :]))
        )
        temp = np.dot(R_s, J)
        R_yInv[i, i] = 1.0 / np.dot(temp, J)

    Pn = np.linalg.inv(np.dot(np.dot(H.T, R_yInv), H))
    F = -((np.dot(n, n) - 1) ** (-0.5)) * np.dot(
        np.linalg.inv(B), np.eye(3) - np.outer(n, n) / (np.dot(n, n) - 1)
    )
    Covar_C_test = np.dot(np.dot(F, Pn), F.T)
    errorNorm1 = np.linalg.norm(n_test - n)
    if errorNorm1 > 1.0e-8:
        print(errorNorm1, "Back Sub")
        testFailCount += 1
        testMessages.append("Back Sub Failure in update test " + "\n")

    r_BN_C = -((np.dot(n, n) - 1.0) ** (-0.5)) * np.dot(np.linalg.inv(B), n)

    posErr = 1e-3  # (m)
    covarErr = 1e-5
    unitTestSupport.writeTeXSnippet("toleranceValuePos", str(posErr), path)
    unitTestSupport.writeTeXSnippet("toleranceValueVel", str(covarErr), path)

    outputR = dataLog.r_BN_C
    outputCovar = dataLog.covar_C
    outputTime = dataLog.timeTag

    for i in range(len(outputR[-1, 1:])):
        if np.abs((r_BN_C[i] - outputR[0, i]) / r_BN_C[i]) > posErr or np.isnan(
            outputR.any()
        ):
            testFailCount += 1
            testMessages.append(
                "FAILED: Position Check in Horizon Nav for index "
                + str(i)
                + " with error "
                + str(np.abs((r_BN_C[i] - outputR[-1, i + 1]) / r_BN_C[i]))
            )

    for i in range(len(outputCovar[-1, 1:])):
        if np.abs(
            (Covar_C_test.flatten()[i] - outputCovar[0, i]) / Covar_C_test.flatten()[i]
        ) > covarErr or np.isnan(outputTime.any()):
            testFailCount += 1
            testMessages.append(
                "FAILED: Covar Check in Horizon Nav for index "
                + str(i)
                + " with error "
                + str(np.abs((Covar_C_test.flatten()[i] - outputCovar[-1, i + 1])))
            )

    snippentName = "passFail"
    if testFailCount == 0:
        colorText = "ForestGreen"
        print("PASSED: " + opNav.ModelTag)
        passedText = r"\textcolor{" + colorText + "}{" + "PASSED" + "}"
    else:
        colorText = "Red"
        print("Failed: " + opNav.ModelTag)
        passedText = r"\textcolor{" + colorText + "}{" + "Failed" + "}"
        print(testMessages)
    unitTestSupport.writeTeXSnippet(snippentName, passedText, path)

    return [testFailCount, "".join(testMessages)]


if __name__ == "__main__":
    # horizonOpNav_methods()
    horizonOpNav_update()
