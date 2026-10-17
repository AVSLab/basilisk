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
from Basilisk.simulation import spacecraft
from Basilisk.simulation import extForceTorque
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import simHelpers
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

def setNoiseAndBounds(
    self,
    senRotNoiseStd=0.0,
    senTransNoiseStd=0.0,
    errorBoundsGyro=[1e6] * 3,  # [rad/s]
    errorBoundsAccel=[1e6] * 3,  # [m/s^2]
):
    # Set the process-noise matrices and optional hard bounds for the IMU module.
    self.PMatrixAccel = np.eye(3) * senTransNoiseStd
    self.setErrorBoundsAccel(np.array(errorBoundsAccel))

    # Set up gyro noise parameters
    self.PMatrixGyro = np.eye(3) * senRotNoiseStd
    self.setErrorBoundsGyro(np.array(errorBoundsGyro))

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
                        (False,         'noise',        0.2,            0.001,      0.0,            0.0,            1000.,          1000.,          .1,                 .1,                 0.3,                0.3,                0.0,            0.0,            1.5e-1),
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
    setNoiseAndBounds(
        ImuSensor,
        senRotNoiseStd,
        senTransNoiseStd,
        errorBoundsGyro,
        errorBoundsAccel,
    )
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
    StateCurrent.v_BN_N = rDot_BN_N[0][:]
    StateCurrent.v_CN_N = rDot_CN_N[0][:]
    # The accumulated delta v fields are built the way spacecraft builds them. No gravity acts in this test, so
    # all the velocity change is non-conservative. TotalAccumDV_BN_B adds the delta v of each step in the body
    # frame of that step, so it must not be used to compute the delta v between IMU calls.
    StateCurrent.TotalAccumDV_CN_N = np.array([0., 0., 0.])  # [m/s]
    StateCurrent.TotalAccumDV_BN_B = np.array([0., 0., 0.])  # [m/s]
    totalAccumDV_BN_B = np.array([0., 0., 0.])  # [m/s]

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
        StateCurrent.v_BN_N = rDot_BN_N[i][:]
        StateCurrent.v_CN_N = rDot_CN_N[i][:]
        StateCurrent.TotalAccumDV_CN_N = rDot_CN_N[i][:] - rDot_CN_N[0][:]
        totalAccumDV_BN_B = totalAccumDV_BN_B + np.dot(dcm_BN_2, rDot_BN_N[i][:] - rDot_BN_N[i-1][:])
        StateCurrent.TotalAccumDV_BN_B = totalAccumDV_BN_B
        scStateMsg.write(StateCurrent, unitSim.TotalSim.CurrentNanos)

    # Pull output time histories from messaging system
    DRout = dataLog.DRFramePlatform
    omegaOut = dataLog.AngVelPlatform
    rDotDotOut = dataLog.AccelPlatform
    DVout = dataLog.DVFramePlatform

    plt.close('all')
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
    simHelpers.writeFigureLaTeX(testCase + "PRVcomparison",
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
    simHelpers.writeFigureLaTeX(testCase + "omegaComparison",
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
    simHelpers.writeFigureLaTeX(testCase + "accelComparison",
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
    simHelpers.writeFigureLaTeX(testCase + "DVcomparison",
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

        # Skip the startup samples and difference consecutive measurements to
        # remove the slowly varying truth signal.
        DRoutNoise = np.diff(DRout[:, 1:], axis=0)[2:]
        rDotDotOutNoise = np.diff(rDotDotOut[:, 1:], axis=0)[2:]
        DVoutNoise = np.diff(DVout[:, 1:], axis=0)[2:]
        omegaOutNoise = np.diff(omegaOut[:, 1:], axis=0)[2:]

        # The difference of two independent white-noise samples has sqrt(2)
        # times the standard deviation of either sample.
        differenceNoiseScale = np.sqrt(2.0)  # [-]
        noiseAccuracy = 0.25  # [-] relative tolerance

        # Compare noise standard deviations with expected values
        expectedDRNoise = differenceNoiseScale * senRotNoiseStd * dt
        accelScaleArray = np.asarray(accelScale)
        expectedDVNoise = differenceNoiseScale * senTransNoiseStd * dt * accelScaleArray
        expectedAccelNoise = differenceNoiseScale * senTransNoiseStd * accelScaleArray
        expectedOmegaNoise = differenceNoiseScale * senRotNoiseStd
        noiseChecks = [
            (np.std(DRoutNoise[:, 0]), expectedDRNoise, "DRnoise1"),
            (np.std(DRoutNoise[:, 1]), expectedDRNoise, "DRnoise2"),
            (np.std(DRoutNoise[:, 2]), expectedDRNoise, "DRnoise3"),
            (np.std(DVoutNoise[:, 0]), expectedDVNoise[0], "DVnoise1"),
            (np.std(DVoutNoise[:, 1]), expectedDVNoise[1], "DVnoise2"),
            (np.std(DVoutNoise[:, 2]), expectedDVNoise[2], "DVnoise3"),
            (np.std(rDotDotOutNoise[:, 0]), expectedAccelNoise[0], "AccelNoise1"),
            (np.std(rDotDotOutNoise[:, 1]), expectedAccelNoise[1], "AccelNoise2"),
            (np.std(rDotDotOutNoise[:, 2]), expectedAccelNoise[2], "AccelNoise3"),
            (np.std(omegaOutNoise[:, 0]), expectedOmegaNoise, "omegaNoise1"),
            (np.std(omegaOutNoise[:, 1]), expectedOmegaNoise, "omegaNoise2"),
            (np.std(omegaOutNoise[:, 2]), expectedOmegaNoise, "omegaNoise3"),
        ]
        for actual, expected, name in noiseChecks:
            print(f"\nChecking {name}:")
            print(f"  Actual value:   {actual}")
            print(f"  Expected value: {expected}")
            if not unitTestSupport.isDoubleEqualRelative(actual, expected, noiseAccuracy):
                msg = f"FAILED {name}. Expected {expected}, got {actual}. \\\\& &"
                testMessages.append(msg)
                testFailCount += 1
            else:
                print("  ✓ Test passed")

        # noise plots
        noiseTimes = DRout[3:, 0] / 1e9  # [s]
        plt.figure(1000, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.clf()
        plt.plot(noiseTimes, DVoutNoise)
        plt.xlabel("Time[s]")
        plt.ylabel("DV Noise [um/s]")
        plt.title("DV Noise")
        simHelpers.writeFigureLaTeX("DVnoise",
                                         'Plot of DeltaV noise along each component for the noise test.',
                                         plt,
                                         'height=0.7\\textwidth, keepaspectratio', path)
        plt.figure(1001, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.clf()
        plt.plot(noiseTimes, rDotDotOutNoise)
        plt.xlabel("Time[s]")
        plt.ylabel("Acceleration Noise [m/s/s]")
        plt.title("Acceleration Noise")
        simHelpers.writeFigureLaTeX("AccelNoise",
                                         'Plot of acceleration noise along each component for the noise test.',
                                         plt,
                                         'height=0.7\\textwidth, keepaspectratio', path)
        plt.figure(1002, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.clf()
        plt.plot(noiseTimes, DRoutNoise)
        plt.xlabel("Time[s]")
        plt.ylabel("DR Noise [rad]")
        plt.title("DR Noise")
        simHelpers.writeFigureLaTeX("DRnoise",
                                         'Plot of PRV noise along each component for the noise test.',
                                         plt,
                                         'height=0.7\\textwidth, keepaspectratio', path)
        plt.figure(1003, figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
        plt.clf()
        plt.plot(noiseTimes, omegaOutNoise)
        plt.xlabel("Time[s]")
        plt.ylabel("Angular Rate Noise [rad/s]")
        plt.title("Angular Rate Noise")
        simHelpers.writeFigureLaTeX("omegaNoise",
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
    simHelpers.writeTeXSnippet(accuracySnippetName, accuracySnippetContent, path)

    if testFailCount == 0:
        colorText = 'ForestGreen'
        passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
    else:
        colorText = 'Red'
        passedText = r'\textcolor{' + colorText + '}{' + "FAILED" + '}'

    passFailSnippetName = testCase+"passFail"
    passFailSnippetContent = passedText
    simHelpers.writeTeXSnippet(passFailSnippetName, passFailSnippetContent, path)

    snippetName = testCase + "gyroLSB"
    snippetContent = '{:1.0e}'.format(gyroLSBIn)
    simHelpers.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "accelLSB"
    snippetContent = '{:1.0e}'.format(accelLSBIn)
    simHelpers.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "rotMax"
    snippetContent = '{:1.1e}'.format(senRotMaxIn)
    simHelpers.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "transMax"
    snippetContent = '{:1.1e}'.format(senTransMaxIn)
    simHelpers.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "rotNoise"
    snippetContent = '{:0.1f}'.format(senRotNoiseStd)
    simHelpers.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "transNoise"
    snippetContent = '{:0.1f}'.format(senTransNoiseStd)
    simHelpers.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "rotBias"
    snippetContent = '{:1.1e}'.format(senRotBiasIn)
    simHelpers.writeTeXSnippet(snippetName, snippetContent, path)
    snippetName = testCase + "transBias"
    snippetContent = '{:1.1e}'.format(senTransBiasIn)
    simHelpers.writeTeXSnippet(snippetName, snippetContent, path)

    if testFailCount:
        print(testMessages)
    else:
        print("PASSED")

    return [testFailCount, ''.join(testMessages)]


def test_imuDeltaVIgnoresRotationOfAccumulatedDeltaV():
    """
    A rotation of the spacecraft must not turn the delta v accumulated before the last IMU call into a new delta v.

    The input message holds 100 m/s of delta v accumulated before the first IMU call, filled the way spacecraft fills
    it. The spacecraft then rotates by 10 deg without any new delta v, so the IMU must output zero. In a third call
    the spacecraft gains 2 m/s along the inertial y axis, which the IMU must output in the current platform frame.
    """
    imu = imuSensor.ImuSensor()
    imu.setBodyToPlatformDCM(0.3, -0.2, 0.1)  # [rad]
    dcm_PB = rbk.euler3212C([0.3, -0.2, 0.1])

    scState = messaging.SCStatesMsgPayload()
    scState.TotalAccumDV_BN_B = [100.0, 0.0, 0.0]  # [m/s] delta v accumulated with sigma_BN = 0
    scState.TotalAccumDV_CN_N = [100.0, 0.0, 0.0]  # [m/s]
    scStateMsg = messaging.SCStatesMsg().write(scState, 0)
    imu.scStateInMsg.subscribeTo(scStateMsg)
    imu.Reset(0)
    imu.UpdateState(0)

    # 10 deg rotation, no new delta v
    sigma_BN = rbk.PRV2MRP([0.0, 0.0, np.radians(10.0)])
    scState.sigma_BN = sigma_BN
    scStateMsg.write(scState, macros.sec2nano(1.0))
    imu.UpdateState(macros.sec2nano(1.0))
    np.testing.assert_allclose(imu.sensorOutMsg.read().DVFramePlatform, [0.0, 0.0, 0.0], rtol=0.0, atol=1e-12,
                               err_msg="a rotation turned accumulated delta v into a new delta v")

    # 2 m/s of new delta v along the inertial y axis
    deltaV_N = np.array([0.0, 2.0, 0.0])  # [m/s]
    dcm_BN = rbk.MRP2C(sigma_BN)
    scState.TotalAccumDV_CN_N = (np.array([100.0, 0.0, 0.0]) + deltaV_N).tolist()  # [m/s]
    scState.TotalAccumDV_BN_B = (np.array([100.0, 0.0, 0.0]) + dcm_BN @ deltaV_N).tolist()  # [m/s]
    scStateMsg.write(scState, macros.sec2nano(2.0))
    imu.UpdateState(macros.sec2nano(2.0))
    np.testing.assert_allclose(imu.sensorOutMsg.read().DVFramePlatform, dcm_PB @ dcm_BN @ deltaV_N,
                               rtol=0.0, atol=1e-12, err_msg="new delta v is wrong")


def test_imuResetDiscardsPreviousState():
    """
    After a call to ``Reset()`` the IMU must not difference the new state against the state recorded before the
    reset.

    The IMU runs twice on a state with 100 m/s of accumulated delta v, a rotated attitude and a sensor velocity
    relative to the center of mass. The state is then set back to zero, as after a restart of the simulation, and
    the IMU is reset. The first call after the reset must output a zero message, and the next call must only see
    the delta v accumulated after the reset.
    """
    imu = imuSensor.ImuSensor()
    imu.sensorPos_B = [1.0, 0.0, 0.0]  # [m]

    scState = messaging.SCStatesMsgPayload()
    scState.TotalAccumDV_CN_N = [100.0, 0.0, 0.0]  # [m/s]
    scState.sigma_BN = rbk.PRV2MRP([0.0, 0.0, np.radians(30.0)])
    scState.omega_BN_B = [0.0, 0.0, 0.1]  # [rad/s]
    scStateMsg = messaging.SCStatesMsg().write(scState, 0)
    imu.scStateInMsg.subscribeTo(scStateMsg)
    imu.Reset(0)
    imu.UpdateState(0)
    imu.UpdateState(macros.sec2nano(1.0))

    # restart from a zero state
    scState = messaging.SCStatesMsgPayload()
    scStateMsg.write(scState, 0)
    imu.Reset(0)
    imu.UpdateState(0)
    output = imu.sensorOutMsg.read()
    np.testing.assert_array_equal(output.DVFramePlatform, [0.0, 0.0, 0.0],
                                  err_msg="the first call after a reset used the state from before the reset")
    np.testing.assert_array_equal(output.DRFramePlatform, [0.0, 0.0, 0.0],
                                  err_msg="the first call after a reset used the state from before the reset")

    # 2 m/s of new delta v along the inertial x axis
    scState.TotalAccumDV_CN_N = [2.0, 0.0, 0.0]  # [m/s]
    scStateMsg.write(scState, macros.sec2nano(1.0))
    imu.UpdateState(macros.sec2nano(1.0))
    np.testing.assert_allclose(imu.sensorOutMsg.read().DVFramePlatform, [2.0, 0.0, 0.0], rtol=0.0, atol=1e-12,
                               err_msg="delta v after a reset is wrong")


@pytest.mark.parametrize("r_BcB_B, sensorPos_B, accuracy", [
    ([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], 1e-10),
    ([0.5, 0.0, 0.2], [1.0, 0.5, -0.3], 1e-5),
])
def test_imuDeltaVWithSpacecraft(r_BcB_B, sensorPos_B, accuracy):
    """
    The IMU delta v must match the non-gravitational change of the sensor velocity when it runs slower than spacecraft.

    A spinning spacecraft in LEO is pushed by a constant inertial force and then coasts. The IMU runs every 10 s
    and the dynamics every 1 s. The truth over each IMU interval is the analytic delta v of the force, F/m times the
    burn time in the interval, plus the change of the sensor velocity relative to the center of mass, resolved in the
    current platform frame. With the sensor at the center of mass the output must be zero during the coast.
    The ``accuracy`` [m/s] of the offset case is set by the RK4 integration error of the spacecraft states.

    Args:
        r_BcB_B (list): [m] hub center of mass location in the body frame
        sensorPos_B (list): [m] IMU location in the body frame
        accuracy (float): [m/s] accuracy of the delta v of each IMU interval
    """
    dynTimeStep = 1.0  # [s]
    imuTimeStep = 10.0  # [s]
    burnStart = 100.0  # [s]
    burnStop = 300.0  # [s]
    stopTime = 600.0  # [s]
    force_N = np.array([10.0, 0.0, 0.0])  # [N]
    mass = 500.0  # [kg]

    sim = SimulationBaseClass.SimBaseClass()
    proc = sim.CreateNewProcess("process")
    proc.addTask(sim.CreateNewTask("dynTask", macros.sec2nano(dynTimeStep)), 100)
    proc.addTask(sim.CreateNewTask("imuTask", macros.sec2nano(imuTimeStep)), 50)

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraft"
    scObject.hub.mHub = mass
    scObject.hub.IHubPntBc_B = np.diag([100.0, 100.0, 100.0]).tolist()  # [kg*m^2]
    scObject.hub.r_BcB_B = [[x] for x in r_BcB_B]  # [m]
    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    earth.isCentralBody = True
    gravFactory.addBodiesTo(scObject)
    oe = orbitalMotion.ClassicElements()
    oe.a = 6778.0e3  # [m]
    oe.e = 0.0  # [-]
    oe.i = 0.5  # [rad]
    oe.Omega = 0.1  # [rad]
    oe.omega = 0.0  # [rad]
    oe.f = 0.0  # [rad]
    r_CN_N, v_CN_N = orbitalMotion.elem2rv(earth.mu, oe)
    scObject.hub.r_CN_NInit = r_CN_N  # [m]
    scObject.hub.v_CN_NInit = v_CN_N  # [m/s]
    scObject.hub.omega_BN_BInit = [[0.01], [0.02], [0.03]]  # [rad/s]
    extForce = extForceTorque.ExtForceTorque()
    extForce.ModelTag = "extForce"
    scObject.addDynamicEffector(extForce)
    sim.AddModelToTask("dynTask", scObject)
    sim.AddModelToTask("dynTask", extForce)

    imu = imuSensor.ImuSensor()
    imu.ModelTag = "imu"
    imu.sensorPos_B = sensorPos_B  # [m]
    imu.setBodyToPlatformDCM(0.3, -0.2, 0.1)  # [rad]
    imu.scStateInMsg.subscribeTo(scObject.scStateOutMsg)
    sim.AddModelToTask("imuTask", imu)
    stateLog = scObject.scStateOutMsg.recorder(macros.sec2nano(imuTimeStep))
    imuLog = imu.sensorOutMsg.recorder(macros.sec2nano(imuTimeStep))
    sim.AddModelToTask("imuTask", stateLog)
    sim.AddModelToTask("imuTask", imuLog)

    sim.InitializeSimulation()
    for segmentStop, forceOn in [(burnStart, 0.0), (burnStop, 1.0), (stopTime, 0.0)]:
        extForce.extForce_N = (forceOn * force_N).tolist()  # [N]
        sim.ConfigureStopTime(macros.sec2nano(segmentStop))
        sim.ExecuteSimulation()

    times = stateLog.times() * macros.NANO2SEC  # [s]
    dcm_PB = rbk.euler3212C([0.3, -0.2, 0.1])
    sensorPos_B = np.array(sensorPos_B)  # [m]
    dcm_NB = [rbk.MRP2C(sigma).T for sigma in stateLog.sigma_BN]
    # sensor velocity relative to the center of mass
    v_SC_N = [stateLog.v_BN_N[k] - stateLog.v_CN_N[k] + dcm_NB[k] @ np.cross(stateLog.omega_BN_B[k], sensorPos_B)
              for k in range(len(times))]  # [m/s]
    DVout = imuLog.DVFramePlatform
    for k in range(1, len(times)):
        burnTime = max(0.0, min(times[k], burnStop) - max(times[k - 1], burnStart))  # [s]
        DV_SN_N = force_N / mass * burnTime + v_SC_N[k] - v_SC_N[k - 1]  # [m/s]
        DVtruth = dcm_PB @ dcm_NB[k].T @ DV_SN_N
        np.testing.assert_allclose(DVout[k], DVtruth, rtol=0.0, atol=accuracy,
                                   err_msg=f"IMU delta v is wrong at t = {times[k]} s")
        if times[k - 1] >= burnStop and not sensorPos_B.any() and not np.any(r_BcB_B):
            np.testing.assert_allclose(DVout[k], [0.0, 0.0, 0.0], rtol=0.0, atol=accuracy,
                                       err_msg=f"IMU at the center of mass sensed delta v during the coast at t = "
                                               f"{times[k]} s")


# This statement below ensures that the unit test script can be run as a
# stand-along python script
if __name__ == "__main__":
    # unitSimIMU(True,        'noise',        1.0,            0.01,       0.0,            0.0,            1000.,          1000.,          0.0,                0.0,                0.0,                0.0,                0.,             0.,             1e-8)
    unitSimIMU(False,         'noise',        0.2,            0.001,      0.0,            0.0,            1000.,          1000.,          .1,                 .1,                 0.1,                0.1,                0.0,            0.0,            1e-1)
