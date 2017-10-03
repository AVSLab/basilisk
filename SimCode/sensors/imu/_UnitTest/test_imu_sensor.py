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
#   IMU Unit Test
#   Purpose:  Test IMU functions
#   Author:  Scott Carnahan
#   Creation Date:  September 14, 2017
#

# import pytest
import sys, os, inspect
import numpy as np
import pytest
import matplotlib.pyplot as plt
import math

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('Basilisk')
sys.path.append(splitPath[0]+'/Basilisk/modules')
sys.path.append(splitPath[0]+'/Basilisk/PythonModules')

import SimulationBaseClass
import unitTestSupport  # general support file with common unit test functions
import macros
import imu_sensor
import sim_model
import RigidBodyKinematics as rbk

# methods

def m33v3mult(M,v):
    output = [[0.],[0.],[0.]]
    np.reshape(v,(3,1))
    for i in range(0,3):
        for k in range(0,3):
            output[i] += M[i][k]*v[k]
    return np.reshape(np.array(output),(1,3))

def m33m33mult(A,B):
    output = np.zeros(9).reshape((3,3))
    for i in range(0,3):
        for j in range(0,3):
            output[i][j] = np.dot(A[i,:],B[:,j])
    return output

def v3vTmult(v1,v2):
    output = [[0,0,0],[0,0,0],[0,0,0]]
    for i in range(0,3):
        for j in range(0,3):
            output[i][j] = v1[i] * v2[j]
    return output

def skew(vector):
    vector = np.array(vector)
    skew_symmetric = np.array([[0, -vector.item(2), vector.item(1)],
                     [vector.item(2), 0, -vector.item(0)],
                     [-vector.item(1), vector.item(0), 0]])
    return skew_symmetric

def findSigmaDot(sigma, omega):
    sigmaMag = np.linalg.norm(sigma)
    B1 = 1 - sigmaMag ** 2
    BI = np.identity(3)
    sigmaTilde = skew(sigma)
    B2 = np.dot(2, sigmaTilde)
    B3 = np.dot(2, v3vTmult(sigma, sigma))
    B = np.dot(B1, BI) + B2 + B3
    sigmaDot = np.dot(0.25, m33v3mult(B, omega))
    return sigmaDot

def setRandomWalk(self,senRotNoiseStd = 0.0,senTransNoiseStd = 0.0,errorBoundsGyro = [1e6] * 3,errorBoundsAccel = [1e6] * 3):
    # sets the random walk for IRU module
    PMatrixGyro = [0.0] * 3 * 3
    PMatrixGyro[0*3+0] = PMatrixGyro[1*3+1] = PMatrixGyro[2*3+2] = senRotNoiseStd
    PMatrixAccel = [0.0] * 3 * 3
    PMatrixAccel[0*3+0] = PMatrixAccel[1*3+1] = PMatrixAccel[2*3+2] = senTransNoiseStd
    self.PMatrixAccel = sim_model.DoubleVector(PMatrixAccel)
    self.walkBoundsAccel = sim_model.DoubleVector(errorBoundsAccel)
    self.PMatrixGyro = sim_model.DoubleVector(PMatrixGyro)
    self.walkBoundsGyro = sim_model.DoubleVector(errorBoundsGyro)

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed

#The following tests are parameterized and run:
# noise - clean plus some noise - test std dev
# error bounds

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("show_plots,   testCase,       stopTime,       gyroLSBIn,      accelLSBIn,     senRotMaxIn,    senTransMaxIn,  senRotNoiseStd,     senTransNoiseStd,   errorBoundsGyroIn,  errorBoundsAccelIn, senRotBiasIn,   senTransBiasIn, accuracy", [
                        (False,          'clean',       1.0,            0.0,            0.0,            1000.,          1000.,          0.0,                0.0,                0.0,                0.0,                0.,             0.,             1e-3),
                        (False,         'noise',        5.0,            0.0,            0.0,            1000.,          1000.,          .1,                 .1,                 0.1,                0.1,                0.0,            0.0,            1e-1),
                        (False,         'bias',         1.0,            0.0,            0.0,            1000.,          1000.,          0.0,                0.0,                0.0,                0.0,                10.,            10.,            1e-3),
                        (False,         'saturation',   1.0,            0.0,            0.0,            0.2,            1.1,            0.0,                0.0,                0.0,                0.0,                0.0,            0.0,            1e-3),
                        (False,         'discretization',1.,            0.0001,         0.0001,         100.,           1000.,          0.0,                0.0,                1e6,                1e6,                0.0,            0.0,            1e-3),

])

# provide a unique test method name, starting with test_
def test_unitSimIMU(show_plots,   testCase,       stopTime,       gyroLSBIn,    accelLSBIn,   senRotMaxIn,    senTransMaxIn,  senRotNoiseStd,     senTransNoiseStd,   errorBoundsGyroIn,  errorBoundsAccelIn, senRotBiasIn,   senTransBiasIn, accuracy):
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimIMU(show_plots,   testCase,       stopTime,       gyroLSBIn,    accelLSBIn,   senRotMaxIn,    senTransMaxIn,  senRotNoiseStd,     senTransNoiseStd,   errorBoundsGyroIn,  errorBoundsAccelIn, senRotBiasIn,   senTransBiasIn, accuracy)
    assert testResults < 1, testMessage


def unitSimIMU(show_plots,   testCase,       stopTime,       gyroLSBIn,    accelLSBIn,   senRotMaxIn,    senTransMaxIn,  senRotNoiseStd,     senTransNoiseStd,   errorBoundsGyroIn,  errorBoundsAccelIn, senRotBiasIn,   senTransBiasIn, accuracy):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcName = "TestProcess"  # arbitrary name (don't change)

    # initialize SimulationBaseClass
    unitSim = SimulationBaseClass.SimBaseClass()
    unitSim.TotalSim.terminateSimulation()

    # create the task and specify the integration update time
    unitProcRate_s = 0.001 #does not pass with .0001 due to floating point issues in macros.sec2nano.
    unitProcRate = macros.sec2nano(unitProcRate_s)
    unitProc = unitSim.CreateNewProcess(unitProcName)
    unitTask = unitSim.CreateNewTask(unitTaskName, unitProcRate)
    unitProc.addTask(unitTask)

    # Set-up the fake kinematics vectors
    # Note: No conservative accelerations are used in this test
    # center of mass
    rDotDot_CN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #acceleration of center of mass wrt inertial frame
    rDotDot_CB = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #acceleration of center of mass wrt body frame
    rDot_CN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #velocity of center of mass wrt inertial frame
    r_CN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #position of center of mass wrt to inertial frame
    # body frame
    rDotDot_BN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) # acceleration of body frame wrt to inertial
    rDot_BN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #velocity of body frame wrt to inertial
    r_BN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #position of body frame wrt to inertial
    omegaDot = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #angular acceleration of body frame wrt to inertial
    omega = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) # angular rate of body frame wrt to inertial
    sigmaDot = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #MRP derivative, body wrt to inertial
    sigma = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) # MRP body wrt to inertial
    # sensor
    rDotDot_SN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #sensor sensed acceleration
    rDot_SN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #sensor accumulated DV
    #rDot_SN_P = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #sensor accumulated DV in sensor platform frame coordinates
    r_SN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #sensor position in body frame coordinates
    test_vector = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3))

    # Set initial conditions for fake kinematics vectors
    # Acceleration vectors
    dataRows= np.shape(rDotDot_CN)[0]
    for i in range(0, dataRows): #loops through acceleration vectors so that each element can be set individually (or, hopefully, as a function)
        rDotDot_BN[i][0] = 1.
        rDotDot_BN[i][1] = 1.
        rDotDot_BN[i][2] = 1.
        rDotDot_CB[i][0] = 0.05
        rDotDot_CB[i][1] = 0.025
        rDotDot_CB[i][2] = 0.03
        rDotDot_CN[i][0] = rDotDot_BN[i][0] + rDotDot_CB[i][0]
        rDotDot_CN[i][1] = rDotDot_BN[i][1] + rDotDot_CB[i][1]
        rDotDot_CN[i][2] = rDotDot_BN[i][2] + rDotDot_CB[i][2]
        omegaDot[i][0] = 0.1
        omegaDot[i][1] = 0.5
        omegaDot[i][2] = 0.25
    # Center of Mass
    rDot_CN[0][:] = [0.0, 0.0, 0.0]
    r_CN[0][:] = [10000., 10000., 10000.] # Some arbitrary location in space
    # Body Frame Origin
    rDot_BN[0][:] = [0.0, 0.0, 0.0]
    r_BN[0][:] = [9999.75, 10000., 10000.] # leaves r_CN[0][i] with some offset
    # Body Rotation
    # omegaDot[0][:] = [0., 0., 0.] #unlabeled omega is omegaDot_BN_B
    omega[0][:] = [0.0, 0.0, 0.0] #omega_BN_B
    sigma[0][:] = [0., 0., 0.25] # note that unlabeled sigma is sigma_BN
    sigmaDot[0][:] = findSigmaDot(sigma[0][:], omega[0][:]) # sigmaDot_BN
    # Sensor linear states (note that these initial conditions must be solved as functions of another initial conditions to maintain consistency
    r_SB = np.array([0., 0., 0.5]) #constant. sensor position wrt to body frame origin
    cDotDot = rDotDot_CN[0][:] - rDotDot_BN[0][:]
    cDot = rDot_CN[0][:] - rDot_BN[0][:]
    c =  r_CN[0][:] - r_BN[0][:]
    cPrime = cDot - np.cross(omega[0][:], c)
    cPrimePrime = cDotDot - np.dot(2, np.cross(omega[0][:], cPrime)) - np.cross(omegaDot[0][:], c) - np.cross(omega[0][:], np.cross(omega[0][:], c))
    r_SC = r_BN[0][:] + r_SB - r_CN[0][:]
    rDotDot_SN[0][:] = rDotDot_CN[0][:] - cPrimePrime - np.dot(2, np.cross(omega[0][:], cPrime)) + np.cross(omegaDot[0][:], r_SC) + np.cross(omega[0][:], np.cross(omega[0][:], r_SC))
    rDot_SN[0][:] = rDot_CN[0][:] - cPrime + np.cross(omega[0][:], r_SC)
    r_SN[0][:] = r_SB + r_BN[0][:]

    #Sensor Setup
    ImuSensor = imu_sensor.ImuSensor()
    ImuSensor.ModelTag = "imusensor"
    ImuSensor.sensorPos_B = imu_sensor.DoubleVector(r_SB) #must be set by user - no default. check if this works by giving an array - SJKC
    yaw = 0.61 #should be given as parameter [rad]
    pitch = 0.33  # [rad]
    roll = 0.75 # [rad]
    dcm_PB = rbk.euler3212C([yaw,pitch,roll]) #done separately as a check
    ImuSensor.setBodyToPlatformDCM(yaw, pitch, roll) # done separately as a check
    errorBoundsGyro = [errorBoundsGyroIn] * 3
    errorBoundsAccel = [errorBoundsAccelIn] * 3
    setRandomWalk(ImuSensor, senRotNoiseStd, senTransNoiseStd, errorBoundsGyro, errorBoundsAccel)
    ImuSensor.gyroLSB = gyroLSBIn
    ImuSensor.accelLSB = accelLSBIn
    ImuSensor.senRotBias = [senRotBiasIn] * 3
    ImuSensor.senTransBias = [senTransBiasIn] * 3
    ImuSensor.senTransMax = senTransMaxIn
    ImuSensor.senRotMax = senRotMaxIn

    # Set-up the sensor output truth vectors
    rDotDot_SN_P = np.resize(np.array([0., 0., 0.]), (int(stopTime/unitProcRate_s+1), 3))  # sensor sensed acceleration in sensor platform frame coordinates
    rDotDot_SN_P[0][:] = m33v3mult(dcm_PB, rDotDot_SN[0][:])
    DVAccum_P = np.resize(np.array([0., 0., 0.]), (int(stopTime/unitProcRate_s+1), 3))  # sensor accumulated delta V ouput in the platform frame
    stepPRV = np.resize(np.array([0., 0., 0.]), (int(stopTime/unitProcRate_s+1), 3))  # principal rotatation vector from time i-1 to time i in platform frame coordinates
    omega_P = np.resize(np.array([0., 0., 0.]), (int(stopTime/unitProcRate_s+1), 3))  # angular rate omega_BN_P = omega_PN_P
    omega_P[0][:] = m33v3mult(dcm_PB, omega[0][:])

    # configure spacecraft dummy message
    StateCurrent = imu_sensor.SCPlusStatesSimMsg()
    StateCurrent.sigma_BN = sigma[0][:]
    StateCurrent.omega_BN_B = omega[0][:] #1 rpm around each axis
    StateCurrent.nonConservativeAccelpntB_B = rDotDot_BN[0][:]
    StateCurrent.omegaDot_BN_B = omegaDot[0][:]
    StateCurrent.TotalAccumDV_BN_B = np.array([0., 0., 0.])

    # add module to the task
    unitSim.AddModelToTask(unitTaskName, ImuSensor)

    # configure inertial_state_output message
    unitSim.TotalSim.CreateNewMessage(unitProcName, "inertial_state_output", 8*3*11, 2)
    unitSim.TotalSim.WriteMessageData("inertial_state_output", 8*3*11, 0, StateCurrent)

    # log module output message
    unitSim.TotalSim.logThisMessage(ImuSensor.OutputDataMsg, unitProcRate)

    unitSim.InitializeSimulation()

    # loop through ExecuteSimulation() and propagate sigma, omega, DV
    dt = unitProcRate_s
    for i in range(1,int(stopTime/dt)+1):
        # Step through the sim
        unitSim.ConfigureStopTime(macros.sec2nano(unitProcRate_s*i))
        unitSim.ExecuteSimulation()

        # linear kinematcs
        rDot_CN[i][:] = rDot_CN[i-1][:] + ((rDotDot_CN[i-1][:] + rDotDot_CN[i][:])/2)*dt
        r_CN[i][:] = r_CN[i-1][:] + ((rDot_CN[i-1][:] + rDot_CN[i][:])/2)*dt

        rDot_BN[i][:] = rDot_BN[i-1][:] + ((rDotDot_BN[i-1][:] + rDotDot_BN[i][:])/2)*dt
        r_BN[i][:] = r_BN[i-1][:] + ((rDot_BN[i-1][:] + rDot_BN[i][:])/2)*dt

        cDotDot = rDotDot_CN[i][:] - rDotDot_BN[i][:]
        cDot = rDot_CN[i][:] - rDot_BN[i][:]
        c = r_CN[i][:] - r_BN[i][:]

        # attitude kinematics
        omega[i][:] = omega[i-1][:] + ((omegaDot[i-1][:] + omegaDot[i][:])/2)*dt

        # iterate on sigma/sigmaDot
        sigmaDot[i][:] = sigmaDot[i-1][:]
        for j in range(0,5): #Seems to converge after a few iterations
            sigma[i][:] = sigma[i-1][:] + ((sigmaDot[i-1][:]+sigmaDot[i][:])/2)*dt
            sigmaDot[i][:] = findSigmaDot(sigma[i][:],omega[i][:])
        sigma[i][:] = sigma[i-1][:] + ((sigmaDot[i-1][:]+sigmaDot[i][:])/2)*dt
        test_vector[i][1] = np.linalg.norm(sigma[i][:])

        # center of mass calculations
        cPrime = cDot - np.cross(omega[i][:], c)
        cPrimePrime = cDotDot - np.dot(2,np.cross(omega[i][:], cPrime)) - np.cross(omegaDot[i][:],c)-np.cross(omega[i][:],np.cross(omega[i][:],c))
        r_SC = r_BN[i][:] + r_SB - r_CN[i][:]

        # solving for sensor inertial states
        rDotDot_SN[i][:] = rDotDot_CN[i][:] - cPrimePrime - np.dot(2,np.cross(omega[i][:],cPrime)) + np.cross(omegaDot[i][:],r_SC) +np.cross(omega[i][:],np.cross(omega[i][:],r_SC))
        rDot_SN[i][:] = rDot_CN[i][:] - cPrime + np.cross(omega[i][:],  r_SC)
        # rDot_SN_check = rDot_SN[i-1][:] + ((rDotDot_SN[i-1][:]+rDotDot_SN[i][:])/2)*dt #This is here to check the output of the "truth" code written here in python if desired
        r_SN[i][:] = r_SN[i-1][:] + ((rDot_SN[i-1][:] + rDot_SN[i][:])/2)*dt #for a simple check of the "truth" code
        # r_SN_simple = r_SC + r_CN[i][:] #for a simple check of the "truth" code if desired.

        # Now create outputs which are (supposed to be) equivalent to the IMU output
        # linear acceleration (non-conservative) in platform frame
        rDotDot_SN_P[i][:] = m33v3mult(dcm_PB, rDotDot_SN[i][:]) + senTransBiasIn #This should match trueValues.AccelPlatform
        # accumulated delta v (non-conservative) in platform frame
        DVAccum_P[i][:] = m33v3mult(dcm_PB, rDot_SN[i][:]-rDot_SN[i-1][:]) + senTransBiasIn*dt
        # find PRV between before and now
        dcm_BN_2 = rbk.MRP2C(sigma[i][:])
        dcm_BN_1 = rbk.MRP2C(sigma[i-1][:])
        dcm_PN_2 = m33m33mult(dcm_PB, dcm_BN_2)
        dcm_PN_1 = m33m33mult(dcm_PB, dcm_BN_1)
        dcm_NP_1 = np.transpose(dcm_PN_1)
        dcm_PN_21 = m33m33mult(dcm_PN_2, dcm_NP_1)
        stepPRV[i][:] = rbk.MRP2PRV(rbk.C2MRP(dcm_PN_21)) + senRotBiasIn*dt
        # angular rate in platform frame
        omega_P[i][:] = m33v3mult(dcm_PB, omega[i][:]) + senRotBiasIn
        #
        # #discretization
        if accelLSBIn > 0.0:
            for k in [0,1,2]:
                discretized_value = np.floor(np.abs(rDotDot_SN_P[i][k] / accelLSBIn)) * accelLSBIn * np.sign(rDotDot_SN_P[i][k])
                discretization_error = rDotDot_SN_P[i][k] - discretized_value
                rDotDot_SN_P[i][k] = discretized_value
                DVAccum_P[i][k] -= discretization_error*dt
                DVAccum_P[i][k] = np.floor(np.abs(DVAccum_P[i][k] / accelLSBIn)) * accelLSBIn * np.sign(DVAccum_P[i][k])
        if gyroLSBIn > 0.0:
            for k in [0,1,2]:
                discretized_value = np.floor(np.abs(omega_P[i][k] / gyroLSBIn)) * gyroLSBIn * np.sign(omega_P[i][k])
                discretization_error = omega_P[i][k] - discretized_value
                omega_P[i][k] = discretized_value
                stepPRV[i][k] -= discretization_error*dt
                stepPRV[i][k] = np.floor(np.abs(stepPRV[i][k] / gyroLSBIn)) * gyroLSBIn * np.sign(stepPRV[i][k])

        #saturation
        for k in [0,1,2]:
            if omega_P[i][k] > senRotMaxIn:
                omega_P[i][k] = senRotMaxIn
                stepPRV[i][k] = senRotMaxIn*dt
            elif omega_P[i][k] < -senRotMaxIn:
                omega_P[i][k] = -senRotMaxIn
                stepPRV[i][k] = -senRotMaxIn*dt
            if rDotDot_SN_P[i][k] > senTransMaxIn:
                rDotDot_SN_P[i][k] = senTransMaxIn
                DVAccum_P[i][k] = senTransMaxIn*dt
            elif rDotDot_SN_P[i][k] < -senTransMaxIn:
                rDotDot_SN_P[i][k] = -senTransMaxIn
                DVAccum_P[i][k] = -senTransMaxIn*dt

        #Now update spacecraft states for the IMU:
        StateCurrent = imu_sensor.SCPlusStatesSimMsg()
        StateCurrent.sigma_BN = sigma[i][:]
        StateCurrent.omega_BN_B = omega[i][:]
        StateCurrent.nonConservativeAccelpntB_B = rDotDot_BN[i][:]
        StateCurrent.omegaDot_BN_B = omegaDot[i][:]
        StateCurrent.TotalAccumDV_BN_B = rDot_BN[i][:] - rDot_BN[0][:]
        unitSim.TotalSim.WriteMessageData("inertial_state_output", 8 * 3 * 11, unitSim.TotalSim.CurrentNanos, StateCurrent)

    DRout       = unitSim.pullMessageLogData(ImuSensor.OutputDataMsg + '.' + "DRFramePlatform", range(3))
    omegaOut    = unitSim.pullMessageLogData(ImuSensor.OutputDataMsg + '.' + "AngVelPlatform", range(3))
    rDotDotOut  = unitSim.pullMessageLogData(ImuSensor.OutputDataMsg + '.' + "AccelPlatform", range(3))
    DVout       = unitSim.pullMessageLogData(ImuSensor.OutputDataMsg + '.' + "DVFramePlatform", range(3))

    plt.figure(1,figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.clf()
    plt.plot(DRout[1:,0], DRout[1:,1], linewidth = 6, color = 'black', label = "output1")
    plt.plot(DRout[:,0], stepPRV[1:,0], linestyle = '--', color = 'white', label = "truth1")
    plt.plot(DRout[1:,0], DRout[1:,2], linewidth = 4, color = 'black', label = "output2")
    plt.plot(DRout[:,0], stepPRV[1:,1], linestyle = '--', color = 'white', label = "truth2")
    plt.plot(DRout[1:,0], DRout[1:,3], linewidth = 2, color = 'black', label = "output3")
    plt.plot(DRout[:,0], stepPRV[1:,2], linestyle = '--', color = 'white', label = "truth3")
    plt.xlabel("Time[ns]")
    plt.ylabel("Time Step PRV Component Magnitude [rad]")
    plt.title("PRV Comparison")
    myLegend = plt.legend()
    myLegend.get_frame().set_facecolor('#909090')
    unitTestSupport.writeFigureLaTeX("PRVcomparison",
                                     'Plot Comparing Time Step PRV Truth and Output. Note that 1, 2, and 3 indicate the components of the principal rotation vector.', plt,
                                     'height=0.7\\textwidth, keepaspectratio', path)
    plt.figure(4,figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.clf()
    plt.plot(DRout[1:,0], omegaOut[1:,1], linewidth = 6, color = 'black', label = "output1")
    plt.plot(DRout[:,0], omega_P[1:,0], linestyle = '--', color = 'white', label = "truth1")
    plt.plot(DRout[1:,0], omegaOut[1:,2], linewidth = 4, color = 'black', label = "output2")
    plt.plot(DRout[:,0], omega_P[1:,1], linestyle = '--', color = 'white', label = "truth2")
    plt.plot(DRout[1:,0], omegaOut[1:,3], linewidth = 2, color = 'black', label = "output3")
    plt.plot(DRout[:,0], omega_P[1:,2], linestyle = '--', color = 'white', label = "truth3")
    plt.xlabel("Time[ns]")
    plt.ylabel("Angular Rate Component Magnitudes [rad/s]")
    plt.title("Angular Rate Comparison")
    myLegend = plt.legend()
    myLegend.get_frame().set_facecolor('#909090')
    unitTestSupport.writeFigureLaTeX("omegaComparison",
                                     'Plot Comparing Angular Rate Truth and Output. Note that 1, 2, and 3 indicate the components of the angular rate.', plt,
                                     'height=0.7\\textwidth, keepaspectratio', path)
    plt.figure(7,figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.clf()
    plt.plot(DRout[1:,0], rDotDotOut[1:,1], linewidth = 6, color = 'black', label = "output1")
    plt.plot(DRout[:,0], rDotDot_SN_P[1:,0], linestyle = '--', color = 'white', label = "truth1")
    plt.plot(DRout[1:,0], rDotDotOut[1:,2], linewidth = 4, color = 'black', label = "output2")
    plt.plot(DRout[:,0], rDotDot_SN_P[1:,1], linestyle = '--', color = 'white', label = "truth2")
    plt.plot(DRout[1:,0], rDotDotOut[1:,3], linewidth = 2, color = 'black', label = "output3")
    plt.plot(DRout[:,0], rDotDot_SN_P[1:,2], linestyle = '--', color = 'white', label = "truth3")
    plt.xlabel("Time[ns]")
    plt.ylabel("Linear Acceleration Component Magnitudes [m/s/s]")
    plt.title("Acceleration Comparison")
    myLegend = plt.legend()
    myLegend.get_frame().set_facecolor('#909090')
    unitTestSupport.writeFigureLaTeX("accelComparison",
                                     'Plot Comparing Sensor Linear Accelertaion Truth and Output. Note that 1, 2, and 3 indicate the components of the acceleration.', plt,
                                     'height=0.7\\textwidth, keepaspectratio', path)
    plt.figure(10,figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.clf()
    plt.plot(DRout[1:,0], DVout[1:,1], linewidth = 6, color = 'black', label = "output1")
    plt.plot(DRout[:,0], DVAccum_P[1:,0], linestyle = '--', color = 'white', label = "truth1")
    plt.plot(DRout[1:,0], DVout[1:,2], linewidth = 4, color = 'black', label = "output2")
    plt.plot(DRout[:,0], DVAccum_P[1:,1], linestyle = '--', color = 'white', label = "truth2")
    plt.plot(DRout[1:,0], DVout[1:,3], linewidth = 2, color = 'black', label = "output3")
    plt.plot(DRout[:,0], DVAccum_P[1:,2], linestyle = '--', color = 'blue', label = "truth3")
    plt.xlabel("Time[ns]")
    plt.ylabel("Step DV Magnitudes [m/s]")
    plt.title("DV Comparison")
    myLegend = plt.legend()
    myLegend.get_frame().set_facecolor('#909090')
    unitTestSupport.writeFigureLaTeX("DVcomparison",
                                     'Plot Comparing Time Step DV Truth and Output. Note that 1, 2, and 3 indicate the components of the velocity delta.', plt,
                                     'height=0.7\\textwidth, keepaspectratio', path)

    if show_plots:
        plt.show()

    # test outputs
    if testCase != 'noise':
        for i in range(2,len(stepPRV)-1):
            if not unitTestSupport.isArrayEqualRelative(DRout[i][:], stepPRV[i][:], 3, accuracy):
                testMessages.append("FAILED DR @ i = "+ str(i) + ". \\\\& &")
                testFailCount += 1
            if not unitTestSupport.isArrayEqualRelative(omegaOut[i][:], omega_P[i][:], 3, accuracy):
                testMessages.append("FAILED OMEGA @ i = "+ str(i) + ". \\\\& &")
                testFailCount += 1
            if not unitTestSupport.isArrayEqualRelative(DVout[i][:], DVAccum_P[i][:], 3, accuracy):
                testMessages.append("FAILED DV @ i = " + str(i) + ". \\\\& &")
                testFailCount += 1
            if not unitTestSupport.isArrayEqualRelative(rDotDotOut[i][:], rDotDot_SN_P[i][:], 3, accuracy):
                testMessages.append("FAILED ACCEL @ i = " + str(i) + ". \\\\& &")
                testFailCount += 1
    else:
        DRoutNoise = np.zeros((np.shape(DRout)[0], np.shape(DRout)[1]-1))
        for i in range(2,len(stepPRV)-1):
            for j in [0,1,2]:
                DRoutNoise[i][j] = DRout[i][j+1] - stepPRV[i+1][j]
        rDotDotOut_Noise = np.zeros((np.shape(DRout)[0], np.shape(DRout)[1] - 1))
        for i in range(2, len(step) - 1):
            for j in [0, 1, 2]:
                rDotDotOut_Noise[i][j] = rDotDotOut[i][j + 1] - rDotDot_SN_P[i+1][j]

    #
    # Outputs to AutoTex
    #
    accuracySnippetName = "accuracy"
    accuracySnippetContent = '{:1.1e}'.format(accuracy)
    unitTestSupport.writeTeXSnippet(accuracySnippetName, accuracySnippetContent, path)

    if testFailCount == 0:
        colorText = 'ForestGreen'
        passedText = '\\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        passedText = '\\textcolor{' + colorText + '}{' + "FAILED" + '}'

    passFailSnippetName = "passFail"
    passFailSnippetContent = passedText
    unitTestSupport.writeTeXSnippet(passFailSnippetName, passFailSnippetContent, path)

    failMsgSnippetName = "failMessage"
    failMsgSnippetContent = ""
    for i in range(0,len(testMessages)):
        failMsgSnippetContent += testMessages[i]
    unitTestSupport.writeTeXSnippet(failMsgSnippetName, failMsgSnippetContent, path)


    return [testFailCount, ''.join(testMessages)]

# This statement below ensures that the unit test script can be run as a
# stand-along python script
if __name__ == "__main__":
    unitSimIMU(True,          'clean',        1.,           0.0,            0.0,            0.1,          1000.,          0.0,                0.0,                0.0,                0.0,                0.0,            0.0,            1e-10)
