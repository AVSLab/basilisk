
# ISC License
#
# Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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


#
#   IMU Unit Test
#   Purpose:  Test IMU functions
#   Author:  Scott Carnahan
#   Creation Date:  September 14, 2017
#

# import pytest
import inspect
import os

import matplotlib.pyplot as plt
import numpy as np
import pytest

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import macros
from Basilisk.simulation import imuSensor
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging

def addTimeColumn(time, data):
    return np.transpose(np.vstack([[time], np.transpose(data)]))

# methods
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
    sigmaDot = np.dot(0.25, np.dot(B, omega))
    return sigmaDot

def setRandomWalk(self,senRotNoiseStd = 0.0,senTransNoiseStd = 0.0,errorBoundsGyro = [1e6] * 3,errorBoundsAccel = [1e6] * 3):
    # sets the random walk for IRU module
    self.PMatrixAccel = np.eye(3) * senRotNoiseStd
    self.walkBoundsAccel = np.array(errorBoundsAccel)
    self.PMatrixGyro = np.eye(3) * senTransNoiseStd
    self.walkBoundsGyro = np.array(errorBoundsGyro)

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed

#The following tests are parameterized and run:
# noise - clean plus some noise - test std dev
# error bounds

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("show_plots,   testCase,       stopTime,       procRate,   gyroLSBIn,      accelLSBIn,     senRotMaxIn,    senTransMaxIn,  senRotNoiseStd,     senTransNoiseStd,   errorBoundsGyroIn,  errorBoundsAccelIn, senRotBiasIn,   senTransBiasIn, accuracy", [
                        (False,         'clean',        1.0,            0.01,       0.0,            0.0,            1000.,          1000.,          0.0,                0.0,                0.0,                0.0,                0.,             0.,             1e-8),
                        (False,         'noise',        1.0,            0.001,      0.0,            0.0,            1000.,          1000.,          .1,                 .1,                 0.1,                0.1,                0.0,            0.0,            1e-1),
                        (False,         'bias',         1.0,            0.01,       0.0,            0.0,            1000.,          1000.,          0.0,                0.0,                0.0,                0.0,                10.,            10.,            1e-8),
                        (False,         'saturation',   1.0,            0.01,       0.0,            0.0,            1.0,            5.0,            0.0,                0.0,                0.0,                0.0,                0.0,            0.0,            1e-8),
                        (False,        'discretization',1.,             0.01,       0.05,           0.5,            100.,           1000.,          0.0,                0.0,                1e6,                1e6,                0.0,            0.0,            1e-8),

])

# provide a unique test method name, starting with test_
def test_unitSimIMU(show_plots,   testCase,       stopTime,       procRate, gyroLSBIn,    accelLSBIn,   senRotMaxIn,    senTransMaxIn,  senRotNoiseStd,     senTransNoiseStd,   errorBoundsGyroIn,  errorBoundsAccelIn, senRotBiasIn,   senTransBiasIn, accuracy):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = unitSimIMU(show_plots,   testCase,       stopTime,       procRate, gyroLSBIn,    accelLSBIn,   senRotMaxIn,    senTransMaxIn,  senRotNoiseStd,     senTransNoiseStd,   errorBoundsGyroIn,  errorBoundsAccelIn, senRotBiasIn,   senTransBiasIn, accuracy)
    assert testResults < 1, testMessage


def unitSimIMU(show_plots,   testCase,       stopTime,       procRate, gyroLSBIn,    accelLSBIn,   senRotMaxIn,    senTransMaxIn,  senRotNoiseStd,     senTransNoiseStd,   errorBoundsGyroIn,  errorBoundsAccelIn, senRotBiasIn,   senTransBiasIn, accuracy):
    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcName = "TestProcess"  # arbitrary name (don't change)

    # initialize SimulationBaseClass
    unitSim = SimulationBaseClass.SimBaseClass()

    # create the task and specify the integration update time
    unitProcRate_s = procRate
    unitProcRate = macros.sec2nano(unitProcRate_s)
    unitProc = unitSim.CreateNewProcess(unitProcName)
    unitTask = unitSim.CreateNewTask(unitTaskName, unitProcRate)
    unitProc.addTask(unitTask)

    # Set-up the fake kinematics vectors
    # Note: No conservative accelerations are used in this test
    # center of mass
    rDotDot_CN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #acceleration of center of mass wrt inertial frame
    rDotDot_CB_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #acceleration of center of mass wrt body frame
    rDot_CN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #velocity of center of mass wrt inertial frame
    r_CN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #position of center of mass wrt to inertial frame
    # body frame
    rDotDot_BN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) # acceleration of body frame wrt to inertial
    rDot_BN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #velocity of body frame wrt to inertial
    r_BN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #position of body frame wrt to inertial
    omegaDot_BN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #angular acceleration of body frame wrt to inertial
    omega_BN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) # angular rate of body frame wrt to inertial
    sigmaDot_BN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #MRP derivative, body wrt to inertial
    sigma_BN = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) # MRP body wrt to inertial
    # sensor
    rDotDot_SN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #sensor sensed acceleration
    rDot_SN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #sensor accumulated DV
    r_SN_N = np.resize(np.array([0., 0., 0.]),(int(stopTime/unitProcRate_s+1),3)) #sensor position in body frame coordinates

    # Set initial conditions for fake kinematics vectors
    # Acceleration vectors
    dataRows= np.shape(rDotDot_CN_N)[0]
    for i in range(0, dataRows): #loops through acceleration vectors so that each element can be set individually (or, hopefully, as a function)
        rDotDot_BN_N[i][0] = 1.
        rDotDot_BN_N[i][1] = 1.
        rDotDot_BN_N[i][2] = 1.
        rDotDot_CB_N[i][0] = 0.05
        rDotDot_CB_N[i][1] = 0.07
        rDotDot_CB_N[i][2] = 0.06
        rDotDot_CN_N[i][0] = rDotDot_BN_N[i][0] + rDotDot_CB_N[i][0]
        rDotDot_CN_N[i][1] = rDotDot_BN_N[i][1] + rDotDot_CB_N[i][1]
        rDotDot_CN_N[i][2] = rDotDot_BN_N[i][2] + rDotDot_CB_N[i][2]
        omegaDot_BN_N[i][0] = 1.
        omegaDot_BN_N[i][1] = 1.5
        omegaDot_BN_N[i][2] = 1.25
    # Center of Mass
    rDot_CN_N[0][:] = [0.05, 0.07, 0.08]
    r_CN_N[0][:] = [10000.5, 10000.7, 10000.5] # Some arbitrary location in space
    # Body Frame Origin
    rDot_BN_N[0][:] = [0.1, 0.2, -0.1]
    r_BN_N[0][:] = [10000., 10000., 10000.] # leaves r_CN_N[0][i] with some offset
    # Body Rotation
    omega_BN_N[0][:] = [0.0, 0.15, 0.1] #omega_BN_N
    sigma_BN[0][:] = [0.25, 0.1, 0.03] # note that unlabeled sigma is sigma_BN
    # Sensor linear states (note that these initial conditions must be solved as functions of another initial conditions to maintain consistency
    r_SB_B = np.array([1.0, 1.0, 2.0]) #constant. sensor position wrt to body frame origin
    cDotDot_N = rDotDot_CN_N[0][:] - rDotDot_BN_N[0][:]
    cDot_N = rDot_CN_N[0][:] - rDot_BN_N[0][:]
    c_N =  r_CN_N[0][:] - r_BN_N[0][:]
    cPrime_N = cDot_N - np.cross(omega_BN_N[0][:], c_N)
    cPrimePrime_N = cDotDot_N - np.dot(2, np.cross(omega_BN_N[0][:], cPrime_N)) - np.cross(omegaDot_BN_N[0][:], c_N) - np.cross(omega_BN_N[0][:], np.cross(omega_BN_N[0][:], c_N))
    dcm_BN = rbk.MRP2C(sigma_BN[0][:])
    dcm_NB = np.transpose(dcm_BN)
    sigmaDot_BN[0][:] = findSigmaDot(sigma_BN[0][:], np.dot(dcm_BN, omega_BN_N[0][:])) # sigmaDot_BN
    r_SB_N = np.dot(dcm_NB, r_SB_B)
    r_SC_N = r_BN_N[0][:] + r_SB_N - r_CN_N[0][:]
    rDotDot_SN_N[0][:] = rDotDot_CN_N[0][:] - cPrimePrime_N - np.dot(2, np.cross(omega_BN_N[0][:], cPrime_N)) + np.cross(omegaDot_BN_N[0][:], r_SC_N) + np.cross(omega_BN_N[0][:], np.cross(omega_BN_N[0][:], r_SC_N))
    rDot_SN_N[0][:] = rDot_CN_N[0][:] - cPrime_N + np.cross(omega_BN_N[0][:], r_SC_N)
    r_SN_N[0][:] = r_SB_N + r_BN_N[0][:]

    #Sensor Setup
    ImuSensor = imuSensor.ImuSensor()
    ImuSensor.ModelTag = "imusensor"
    ImuSensor.sensorPos_B = np.array(r_SB_B) #must be set by user - no default. check if this works by giving an array - SJKC
    yaw = 0.7854 #should be given as parameter [rad]
    pitch = 1.0  # [rad]
    roll = 0.1 # [rad]
    dcm_PB = rbk.euler3212C([yaw,pitch,roll]) #done separately as a
    dcm_PN = np.dot(dcm_PB, dcm_BN)
    ImuSensor.setBodyToPlatformDCM(yaw, pitch, roll) # done separately as a check
    errorBoundsGyro = [errorBoundsGyroIn] * 3
    errorBoundsAccel = [errorBoundsAccelIn] * 3
    setRandomWalk(ImuSensor, senRotNoiseStd, senTransNoiseStd, errorBoundsGyro, errorBoundsAccel)
    ImuSensor.setLSBs(accelLSBIn, gyroLSBIn)
    ImuSensor.senRotBias = np.array([senRotBiasIn] * 3)
    ImuSensor.senTransBias = np.array([senTransBiasIn] * 3)
    ImuSensor.senTransMax = senTransMaxIn
    ImuSensor.senRotMax = senRotMaxIn
    accelScale = [2.,2.,2.]
    gyroScale = [1.,1.,1.]
    ImuSensor.accelScale = np.array(accelScale)
    ImuSensor.gyroScale = np.array(gyroScale)

    accel_SN_P_disc = np.array([0., 0., 0.])
    omega_SN_P_disc = np.array([0., 0., 0.])

    # Set-up the sensor output truth vectors
    rDotDot_SN_P = np.resize(np.array([0., 0., 0.]), (int(stopTime/unitProcRate_s+1), 3))  # sensor sensed acceleration in sensor platform frame coordinates
    rDotDot_SN_P[0][:] = np.dot(dcm_PN, rDotDot_SN_N[0][:])
    DVAccum_SN_P = np.resize(np.array([0., 0., 0.]), (int(stopTime/unitProcRate_s+1), 3))  # sensor accumulated delta V ouput in the platform frame
    stepPRV_PN = np.resize(np.array([0., 0., 0.]), (int(stopTime/unitProcRate_s+1), 3))  # principal rotatation vector from time i-1 to time i in platform frame coordinates
    omega_PN_P = np.resize(np.array([0., 0., 0.]), (int(stopTime/unitProcRate_s+1), 3))  # angular rate omega_BN_P = omega_PN_P
    omega_PN_P[0][:] = np.dot(dcm_PN, omega_BN_N[0][:])

    # configure spacecraft dummy message - Need to convert to B frame here first
    StateCurrent = messaging.SCStatesMsgPayload()
    StateCurrent.sigma_BN = sigma_BN[0][:]
    StateCurrent.omega_BN_B = np.dot(dcm_BN, omega_BN_N[0][:]) #1 rpm around each axis
    StateCurrent.nonConservativeAccelpntB_B = np.dot(dcm_BN, rDotDot_BN_N[0][:])
    StateCurrent.omegaDot_BN_B = np.dot(dcm_BN, omegaDot_BN_N[0][:])
    StateCurrent.TotalAccumDV_BN_B = np.array([0., 0., 0.])

    # add module to the task
    unitSim.AddModelToTask(unitTaskName, ImuSensor)

    # configure inertial_state_output message
    scStateMsg = messaging.SCStatesMsg().write(StateCurrent)
    ImuSensor.scStateInMsg.subscribeTo(scStateMsg)

    # log module output message
    dataLog = ImuSensor.sensorOutMsg.recorder()
    unitSim.AddModelToTask(unitTaskName, dataLog)

    unitSim.InitializeSimulation()

    # loop through ExecuteSimulation() and propagate sigma, omega, DV
    dt = unitProcRate_s
    for i in range(1,int(stopTime/dt)+1):
        # Step through the sim
        unitSim.ConfigureStopTime(macros.sec2nano(unitProcRate_s*i))
        unitSim.ExecuteSimulation()

        # attitude kinematics
        omega_BN_N[i][:] = omega_BN_N[i-1][:] + ((omegaDot_BN_N[i-1][:] + omegaDot_BN_N[i][:])/2)*dt

        # iterate on sigma/sigmaDot
        sigmaDot_BN[i][:] = sigmaDot_BN[i-1][:]
        for j in range(0,10): #Seems to converge after a few iterations
            sigma_BN[i][:] = sigma_BN[i-1][:] + ((sigmaDot_BN[i-1][:]+sigmaDot_BN[i][:])/2)*dt
            dcm_BN_2 = rbk.MRP2C(sigma_BN[i][:])
            sigmaDot_BN[i][:] = findSigmaDot(sigma_BN[i][:],np.dot(dcm_BN_2, omega_BN_N[i][:]))
        sigma_BN[i][:] = sigma_BN[i-1][:] + ((sigmaDot_BN[i-1][:]+sigmaDot_BN[i][:])/2)*dt
        dcm_BN_2 = rbk.MRP2C(sigma_BN[i][:])
        dcm_NB = np.transpose(dcm_BN_2)
        r_SB_N = np.dot(dcm_NB, r_SB_B)

        # linear kinematcs
        rDot_CN_N[i][:] = rDot_CN_N[i-1][:] + ((rDotDot_CN_N[i-1][:] + rDotDot_CN_N[i][:])/2)*dt
        r_CN_N[i][:] = r_CN_N[i-1][:] + ((rDot_CN_N[i-1][:] + rDot_CN_N[i][:])/2)*dt

        rDot_BN_N[i][:] = rDot_BN_N[i-1][:] + ((rDotDot_BN_N[i-1][:] + rDotDot_BN_N[i][:])/2)*dt
        r_BN_N[i][:] = r_BN_N[i-1][:] + ((rDot_BN_N[i-1][:] + rDot_BN_N[i][:])/2)*dt

        cDotDot_N = rDotDot_CN_N[i][:] - rDotDot_BN_N[i][:]
        cDot_N = rDot_CN_N[i][:] - rDot_BN_N[i][:]
        c_N = r_CN_N[i][:] - r_BN_N[i][:]

        # center of mass calculations
        cPrime_N = cDot_N - np.cross(omega_BN_N[i][:], c_N)
        cPrimePrime_N = cDotDot_N - np.dot(2,np.cross(omega_BN_N[i][:], cPrime_N)) - np.cross(omegaDot_BN_N[i][:],c_N)-np.cross(omega_BN_N[i][:],np.cross(omega_BN_N[i][:],c_N))
        r_SC_N = r_BN_N[i][:] + r_SB_N - r_CN_N[i][:]

        # solving for sensor inertial states
        rDotDot_SN_N[i][:] = rDotDot_CN_N[i][:] - cPrimePrime_N - np.dot(2,np.cross(omega_BN_N[i][:],cPrime_N)) + np.cross(omegaDot_BN_N[i][:],r_SC_N) +np.cross(omega_BN_N[i][:],np.cross(omega_BN_N[i][:],r_SC_N))
        rDot_SN_N[i][:] = rDot_CN_N[i][:] - cPrime_N + np.cross(omega_BN_N[i][:],r_SC_N)

        # Now create outputs which are (supposed to be) equivalent to the IMU output
        # linear acceleration (non-conservative) in platform frame
        dcm_BN_1 = rbk.MRP2C(sigma_BN[i-1][:])
        dcm_PN_2 = np.dot(dcm_PB, dcm_BN_2)
        dcm_PN_1 = np.dot(dcm_PB, dcm_BN_1)
        dcm_NP_1 = np.transpose(dcm_PN_1)
        dcm_PN_21 = np.dot(dcm_PN_2, dcm_NP_1)
        rDotDot_SN_P[i][:] = np.multiply(np.dot(dcm_PN_2, rDotDot_SN_N[i][:]) + senTransBiasIn, accelScale) #This should match trueValues.AccelPlatform
        # accumulated delta v (non-conservative) in platform frame
        DVAccum_SN_P[i][:] = np.multiply(np.dot(dcm_PN_2, rDot_SN_N[i][:] - rDot_SN_N[i-1][:]) + senTransBiasIn*dt, accelScale)

        # find PRV between before and now

        stepPRV_PN[i][:] = np.multiply(rbk.MRP2PRV(rbk.C2MRP(dcm_PN_21)) + senRotBiasIn*dt, gyroScale)

        # angular rate in platform frame
        omega_PN_P[i][:] = np.multiply(np.dot(dcm_PN_2, omega_BN_N[i][:]) + senRotBiasIn, gyroScale)
        #
        # #discretization
        if accelLSBIn > 0.0:
            for k in [0,1,2]:
                accel_SN_P_disc[k] = np.floor(np.abs(rDotDot_SN_P[i][k] / accelLSBIn)) * accelLSBIn * np.sign(rDotDot_SN_P[i][k])
            accelDiscError = rDotDot_SN_P[i][:] - accel_SN_P_disc
            rDotDot_SN_P[i][:] = accel_SN_P_disc
            DVAccum_SN_P[i][:] -= accelDiscError * dt
        if gyroLSBIn > 0.0:
            for k in [0,1,2]:
                omega_SN_P_disc[k] = np.floor(np.abs(omega_PN_P[i][k] / gyroLSBIn)) * gyroLSBIn * np.sign(omega_PN_P[i][k])
            omegaDiscError = omega_PN_P[i][:] - omega_SN_P_disc
            omega_PN_P[i][:] = omega_SN_P_disc
            stepPRV_PN[i][:] -= omegaDiscError * dt

        #saturation
        for k in [0,1,2]:
            if omega_PN_P[i][k] > senRotMaxIn:
                omega_PN_P[i][k] = senRotMaxIn
                stepPRV_PN[i][k] = senRotMaxIn*dt
            elif omega_PN_P[i][k] < -senRotMaxIn:
                omega_PN_P[i][k] = -senRotMaxIn
                stepPRV_PN[i][k] = -senRotMaxIn*dt
            if rDotDot_SN_P[i][k] > senTransMaxIn:
                rDotDot_SN_P[i][k] = senTransMaxIn
                DVAccum_SN_P[i][k] = rDotDot_SN_P[i][k] * dt
            elif rDotDot_SN_P[i][k] < -senTransMaxIn:
                rDotDot_SN_P[i][k] = -senTransMaxIn
                DVAccum_SN_P[i][k] = rDotDot_SN_P[i][k] * dt

        #Now update spacecraft states for the IMU:
        StateCurrent = messaging.SCStatesMsgPayload()
        StateCurrent.sigma_BN = sigma_BN[i][:]
        StateCurrent.omega_BN_B = np.dot(dcm_BN_2, omega_BN_N[i][:])
        StateCurrent.nonConservativeAccelpntB_B = np.dot(dcm_BN_2, rDotDot_BN_N[i][:])
        StateCurrent.omegaDot_BN_B = np.dot(dcm_BN_2, omegaDot_BN_N[i][:])
        StateCurrent.TotalAccumDV_BN_B = np.dot(dcm_BN_2, rDot_BN_N[i][:] - rDot_BN_N[0][:])
        scStateMsg.write(StateCurrent, unitSim.TotalSim.CurrentNanos)

    # Pull output time histories from messaging system
    DRout = dataLog.DRFramePlatform
    omegaOut = dataLog.AngVelPlatform
    rDotDotOut = dataLog.AccelPlatform
    DVout = dataLog.DVFramePlatform

    # truth/output comparison plots and AutoTex output
    time = dataLog.times()/1e9
    plt.figure(1,figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.clf()
    plt.plot(time, DRout[0:,0], linewidth = 6, color = 'black', label = "output1")
    plt.plot(time, stepPRV_PN[:,0], linestyle = '--', color = 'cyan', label = "truth1")
    plt.plot(time, DRout[0:,1], linewidth = 4, color = 'black', label = "output2")
    plt.plot(time, stepPRV_PN[:,1], linestyle = '--', color = 'cyan', label = "truth2")
    plt.plot(time, DRout[0:,2], linewidth = 2, color = 'black', label = "output3")
    plt.plot(time, stepPRV_PN[:,2], linestyle = '--', color = 'cyan', label = "truth3")
    plt.xlabel("Time[s]")
    plt.ylabel("Time Step PRV Component Magnitude [rad]")
    plt.title("PRV Comparison")
    myLegend = plt.legend()
    myLegend.get_frame().set_facecolor('#909090')
    unitTestSupport.writeFigureLaTeX(testCase + "PRVcomparison",
                                     'Plot Comparing Time Step PRV Truth and Output for test: ' + testCase +'. Note that 1, 2, and 3 indicate the components of the principal rotation vector.', plt,
                                     'height=0.7\\textwidth, keepaspectratio', path)
    plt.figure(4,figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.clf()
    plt.plot(time, omegaOut[:,0], linewidth = 6, color = 'black', label = "output1")
    plt.plot(time, omega_PN_P[:,0], linestyle = '--', color = 'cyan', label = "truth1")
    plt.plot(time, omegaOut[:,1], linewidth = 4, color = 'black', label = "output2")
    plt.plot(time, omega_PN_P[:,1], linestyle = '--', color = 'cyan', label = "truth2")
    plt.plot(time, omegaOut[:,2], linewidth = 2, color = 'black', label = "output3")
    plt.plot(time, omega_PN_P[:,2], linestyle = '--', color = 'cyan', label = "truth3")
    plt.xlabel("Time[s]")
    plt.ylabel("Angular Rate Component Magnitudes [rad/s]")
    plt.title("Angular Rate Comparison")
    myLegend = plt.legend()
    myLegend.get_frame().set_facecolor('#909090')
    unitTestSupport.writeFigureLaTeX(testCase + "omegaComparison",
                                     'Plot Comparing Angular Rate Truth and Output for test: ' + testCase +'. Note that 1, 2, and 3 indicate the components of the angular rate.', plt,
                                     'height=0.7\\textwidth, keepaspectratio', path)
    plt.figure(7,figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.clf()
    plt.plot(time, rDotDotOut[:,0], linewidth = 6, color = 'black', label = "output1")
    plt.plot(time, rDotDot_SN_P[:,0], linestyle = '--', color = 'cyan', label = "truth1")
    plt.plot(time, rDotDotOut[:,1], linewidth = 4, color = 'black', label = "output2")
    plt.plot(time, rDotDot_SN_P[:,1], linestyle = '--', color = 'cyan', label = "truth2")
    plt.plot(time, rDotDotOut[:,2], linewidth = 2, color = 'black', label = "output3")
    plt.plot(time, rDotDot_SN_P[:,2], linestyle = '--', color = 'cyan', label = "truth3")
    plt.xlabel("Time[s]")
    plt.ylabel("Linear Acceleration Component Magnitudes [m/s/s]")
    plt.title("Acceleration Comparison")
    myLegend = plt.legend()
    myLegend.get_frame().set_facecolor('#909090')
    unitTestSupport.writeFigureLaTeX(testCase + "accelComparison",
                                     'Plot Comparing Sensor Linear Accelertaion Truth and Output for test: ' + testCase +'. Note that 1, 2, and 3 indicate the components of the acceleration.', plt,
                                     'height=0.7\\textwidth, keepaspectratio', path)

    plt.figure(10,figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.clf()
    plt.plot(time, DVout[:,0], linewidth = 6, color = 'black', label = "output1")
    plt.plot(time, DVAccum_SN_P[:,0], linestyle = '--', color = 'cyan', label = "truth1")
    plt.plot(time, DVout[:,1], linewidth = 4, color = 'black', label = "output2")
    plt.plot(time, DVAccum_SN_P[:,1], linestyle = '--', color = 'cyan', label = "truth2")
    plt.plot(time, DVout[:,2], linewidth = 2, color = 'black', label = "output3")
    plt.plot(time, DVAccum_SN_P[:,2], linestyle = '--', color = 'cyan', label = "truth3")
    plt.xlabel("Time[s]")
    plt.ylabel("Step DV Magnitudes [m/s]")
    plt.title("DV Comparison")
    myLegend = plt.legend()
    myLegend.get_frame().set_facecolor('#909090')
    unitTestSupport.writeFigureLaTeX(testCase + "DVcomparison",
                                     'Plot Comparing Time Step DV Truth and Output for test: ' + testCase +'. Note that 1, 2, and 3 indicate the components of the velocity delta.', plt,
                                     'height=0.7\\textwidth, keepaspectratio', path)

    if show_plots and testCase != "noise":
        plt.show()
        plt.close('all')

    # test outputs
    if testCase != 'noise':
        for i in range(2,len(stepPRV_PN)-1):
            if not unitTestSupport.isArrayEqualRelative(DRout[i+1][:], stepPRV_PN[i][:], 3, accuracy):
                testMessages.append("FAILED DR @ i = "+ str(i) + ". \\\\& &")
                testFailCount += 1
            if not unitTestSupport.isArrayEqualRelative(omegaOut[i+1][:], omega_PN_P[i][:], 3, accuracy):
                testMessages.append("FAILED OMEGA @ i = "+ str(i) + ". \\\\& &")
                testFailCount += 1
            if not (testCase == "discretization" and (i == 572 or i == 934)):
                if not unitTestSupport.isArrayEqualRelative(DVout[i+1][:], DVAccum_SN_P[i][:], 3, accuracy):
                    testMessages.append("FAILED DV @ i = " + str(i) + ". \\\\& &")
                    testFailCount += 1
            if not unitTestSupport.isArrayEqualRelative(rDotDotOut[i+1][:], rDotDot_SN_P[i][:], 3, accuracy):
                testMessages.append("FAILED ACCEL @ i = " + str(i) + ". \\\\& &")
                testFailCount += 1
    else:
        DRout = addTimeColumn(dataLog.times(), DRout)[1:,]
        rDotDotOut = addTimeColumn(dataLog.times(), rDotDotOut)[1:,]
        DVout = addTimeColumn(dataLog.times(), DVout)[1:,]
        omegaOut = addTimeColumn(dataLog.times(), omegaOut)[1:,]

        DRoutNoise = np.zeros((np.shape(DRout)[0], np.shape(DRout)[1]-1))
        for i in range(3,len(stepPRV_PN)-1):
            for j in [0,1,2]:
                DRoutNoise[i][j] = DRout[i][j+1] - stepPRV_PN[i+1][j]
        rDotDotOutNoise = np.zeros((np.shape(DRout)[0], np.shape(DRout)[1] - 1))
        for i in range(3, len(stepPRV_PN) - 1):
            for j in [0, 1, 2]:
                rDotDotOutNoise[i, j] = rDotDotOut[i, j+1] - rDotDot_SN_P[i+1, j]
        DVoutNoise = np.zeros((np.shape(DRout)[0], np.shape(DRout)[1] - 1))
        for i in range(3, len(stepPRV_PN) - 1):
            for j in [0, 1, 2]:
                DVoutNoise[i, j] = DVout[i, j + 1] - DVAccum_SN_P[i + 1, j]
        omegaOutNoise = np.zeros((np.shape(DRout)[0], np.shape(DRout)[1] - 1))
        for i in range(3, len(stepPRV_PN)-1):
            for j in [0, 1, 2]:
                omegaOutNoise[i, j] = omegaOut[i, j + 1] - omega_PN_P[i + 1, j]

        if not unitTestSupport.isDoubleEqualRelative(np.std(DRoutNoise[:,0]),senRotNoiseStd*dt/1.5,accuracy):
            testMessages.append(("FAILED DRnoise1. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(DRoutNoise[:,1]),senRotNoiseStd*dt/1.5,accuracy):
            testMessages.append(("FAILED DRnoise2. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(DRoutNoise[:,2]),senRotNoiseStd*dt/1.5,accuracy):
            testMessages.append(("FAILED DRnoise3. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(DVoutNoise[:,0]),senTransNoiseStd*dt/1.5 * accelScale[0],accuracy):
            testMessages.append(("FAILED DVnoise1. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(DVoutNoise[:,1]),senTransNoiseStd*dt/1.5 * accelScale[1],accuracy):
            testMessages.append(("FAILED DVnoise2. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(DVoutNoise[:,2]),senTransNoiseStd*dt/1.5 * accelScale[2],accuracy):
            testMessages.append(("FAILED DVnoise3. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(rDotDotOutNoise[:,0]),senTransNoiseStd/1.5 * accelScale[0],accuracy):
            testMessages.append(("FAILED AccelNoise1. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(rDotDotOutNoise[:,1]),senTransNoiseStd/1.5 * accelScale[1],accuracy):
            testMessages.append(("FAILED AccelNoise2. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(rDotDotOutNoise[:,2]),senTransNoiseStd/1.5 * accelScale[2],accuracy):
            testMessages.append(("FAILED AccelNoise3. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(omegaOutNoise[:,0]),senRotNoiseStd/1.5,accuracy):
            testMessages.append(("FAILED omegaNoise1. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(omegaOutNoise[:,1]),senRotNoiseStd/1.5,accuracy):
            testMessages.append(("FAILED oemgaNoise2. \\\\& &"))
            testFailCount += 1
        if not unitTestSupport.isDoubleEqualRelative(np.std(omegaOutNoise[:,2]),senRotNoiseStd/1.5,accuracy):
            testMessages.append(("FAILED omegaNoise3. \\\\& &"))
            testFailCount += 1

        # noise plots
        plt.figure(1000, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.clf()
        plt.plot(DRout[1:, 0]/1e9, DVoutNoise[1:,:])
        plt.xlabel("Time[s]")
        plt.ylabel("DV Noise [um/s]")
        plt.title("DV Noise")
        unitTestSupport.writeFigureLaTeX("DVnoise",
                                         'Plot of DeltaV noise along each component for the noise test.',
                                         plt,
                                         'height=0.7\\textwidth, keepaspectratio', path)
        plt.figure(1001, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.clf()
        plt.plot(DRout[1:, 0]/1e9, rDotDotOutNoise[1:,:])
        plt.xlabel("Time[s]")
        plt.ylabel("Acceleration Noise [m/s/s]")
        plt.title("Acceleration Noise")
        unitTestSupport.writeFigureLaTeX("AccelNoise",
                                         'Plot of acceleration noise along each component for the noise test.',
                                         plt,
                                         'height=0.7\\textwidth, keepaspectratio', path)
        plt.figure(1002, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.clf()
        plt.plot(DRout[1:, 0]/1e9, DRoutNoise[1:,:])
        plt.xlabel("Time[s]")
        plt.ylabel("DR Noise [rad]")
        plt.title("DR Noise")
        unitTestSupport.writeFigureLaTeX("DRnoise",
                                         'Plot of PRV noise along each component for the noise test.',
                                         plt,
                                         'height=0.7\\textwidth, keepaspectratio', path)
        plt.figure(1003, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.clf()
        plt.plot(DRout[1:, 0]/1e9, omegaOutNoise[1:,:])
        plt.xlabel("Time[s]")
        plt.ylabel("Angular Rate Noise [rad/s]")
        plt.title("Angular Rate Noise")
        unitTestSupport.writeFigureLaTeX("omegaNoise",
                                         'Plot of Angular Rate noise along each component for the noise test.',
                                         plt,
                                         'height=0.7\\textwidth, keepaspectratio', path)
        if show_plots:
            plt.show()
            plt.close('all')

    #
    # Outputs to AutoTex
    #
    accuracySnippetName = testCase+"accuracy"
    accuracySnippetContent = '{:1.0e}'.format(accuracy)
    unitTestSupport.writeTeXSnippet(accuracySnippetName, accuracySnippetContent, path)

    if testFailCount == 0:
        colorText = 'ForestGreen'
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        passedText = r'\textcolor{' + colorText + '}{' + "FAILED" + '}'

    passFailSnippetName = testCase+"passFail"
    passFailSnippetContent = passedText
    unitTestSupport.writeTeXSnippet(passFailSnippetName, passFailSnippetContent, path)

    snippetName = testCase + "gyroLSB"
    snippetContent = '{:1.0e}'.format(gyroLSBIn)
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "accelLSB"
    snippetContent = '{:1.0e}'.format(accelLSBIn)
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "rotMax"
    snippetContent = '{:1.1e}'.format(senRotMaxIn)
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "transMax"
    snippetContent = '{:1.1e}'.format(senTransMaxIn)
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "rotNoise"
    snippetContent = '{:0.1f}'.format(senRotNoiseStd)
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "transNoise"
    snippetContent = '{:0.1f}'.format(senTransNoiseStd)
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "rotBias"
    snippetContent = '{:1.1e}'.format(senRotBiasIn)
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "transBias"
    snippetContent = '{:1.1e}'.format(senTransBiasIn)
    unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)

    if testFailCount:
        print(testMessages)
    else:
        print("PASSED")

    return [testFailCount, ''.join(testMessages)]

# This statement below ensures that the unit test script can be run as a
# stand-along python script
if __name__ == "__main__":
    # unitSimIMU(True,        'noise',        1.0,            0.01,       0.0,            0.0,            1000.,          1000.,          0.0,                0.0,                0.0,                0.0,                0.,             0.,             1e-8)
    unitSimIMU(False,         'noise',        1.0,            0.001,      0.0,            0.0,            1000.,          1000.,          .1,                 .1,                 0.1,                0.1,                0.0,            0.0,            1e-1)