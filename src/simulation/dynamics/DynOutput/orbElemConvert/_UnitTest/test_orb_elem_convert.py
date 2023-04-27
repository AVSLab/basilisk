
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


import inspect
import math
import os

import matplotlib.pyplot as plt
import numpy
import pytest

#
# orb_elem_convert Unit Test
#
# Purpose:  Test the precision of the orb_elem_convert module. Functionality
#           is tested by comparing input/output data as well as calculated
#           conversions.
# Author:   Gabriel Chapel
# Creation Date:  July 27, 2017
#

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
splitPath = path.split('simulation')

from Basilisk.utilities import SimulationBaseClass
from Basilisk.simulation import orbElemConvert
from Basilisk.utilities import macros
from Basilisk.utilities import macros as mc
from Basilisk.utilities import unitTestSupport
from Basilisk.architecture import messaging

# Class in order to plot using data across the different parameterized scenarios
class DataStore:
    def __init__(self):
        self.Date = [] # replace these with appropriate containers for the data to be stored for plotting
        self.MarsPosErr = []
        self.EarthPosErr = []
        self.SunPosErr = []


@pytest.mark.parametrize("a, e, i, AN, AP, f, mu, name", [
    # Inclined Elliptical Orbit Varying e
    (10000000.0, 0.01, 33.3*mc.D2R, 48.2*mc.D2R, 347.8*mc.D2R, 85.3*mc.D2R, 0.3986004415E+15, 'IncEllip_e_1'),
    (10000000.0, 0.10, 33.3*mc.D2R, 48.2*mc.D2R, 347.8*mc.D2R, 85.3*mc.D2R, 0.3986004415E+15, 0),
    (10000000.0, 0.25, 33.3*mc.D2R, 48.2*mc.D2R, 347.8*mc.D2R, 85.3*mc.D2R, 0.3986004415E+15, 0),
    (10000000.0, 0.50, 33.3*mc.D2R, 48.2*mc.D2R, 347.8*mc.D2R, 85.3*mc.D2R, 0.3986004415E+15, 0),
    (10000000.0, 0.75, 33.3*mc.D2R, 48.2*mc.D2R, 347.8*mc.D2R, 85.3*mc.D2R, 0.3986004415E+15, 'IncEllip_e_2'),
    # Inclined Elliptical Orbit Varying a
    (10000000.0, 0.50, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'IncEllip_a_1'),
    (100000.0, 0.50, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (10000.0, 0.50, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (1000.0, 0.50, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (100.0, 0.50, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (10.0, 0.50, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'IncEllip_a_2'),

    # Equatorial Elliptical Orbit Varying e
    (10000000.0, 0.01, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'EquEllip_e_1'),
    (10000000.0, 0.10, 0.0, 0.0, 347.8*mc.D2R, 85.3*mc.D2R, 0.3986004415E+15, 0),
    (10000000.0, 0.25, 0.0, 0.0, 347.8*mc.D2R, 85.3*mc.D2R, 0.3986004415E+15, 0),
    (10000000.0, 0.50, 0.0, 0.0, 347.8*mc.D2R, 85.3*mc.D2R, 0.3986004415E+15, 0),
    (10000000.0, 0.75, 0.0, 0.0, 347.8*mc.D2R, 85.3*mc.D2R, 0.3986004415E+15, 'EquEllip_e_2'),
    # Equatorial Elliptical Orbit Varying a
    (10000000.0, 0.50, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'EquEllip_a_1'), # For i=0 => AN=0
    (100000.0, 0.50, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (10000.0, 0.50, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (1000.0, 0.50, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (100.0, 0.50, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (10.0, 0.50, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'EquEllip_a_2'),

    # Inclined Circular Orbit
    (10000000.0, 0.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 'IncCirc_1'),
    (1000000.0, 0.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (100000.0, 0.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (10000.0, 0.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (1000.0, 0.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (100.0, 0.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (10.0, 0.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 'IncCirc_2'),

    # Equatorial Circular Orbit
    (10000000.0, 0.0, 0.0, 0.0, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 'EquCirc_1'),
    (1000000.0, 0.0, 0.0, 0.0, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (100000.0, 0.0, 0.0, 0.0, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (10000.0, 0.0, 0.0, 0.0, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (1000.0, 0.0, 0.0, 0.0, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (100.0, 0.0, 0.0, 0.0, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (10.0, 0.0, 0.0, 0.0, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 'EquCirc_2'),

    # Inclined Parabolic Orbit
    (-10.0, 1.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'IncPara_1'),   # For input of -a,
    (-100.0, 1.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),  # must have e= 1.0
    (-1000.0, 1.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0), # or e >1.0
    (-10000.0, 1.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-100000.0, 1.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'IncPara_2'),

    # Equatorial Parabolic Orbit
    (-10.0, 1.0, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'EquPara_1'),   # For input of -a,
    (-100.0, 1.0, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),  # must have e= 1.0
    (-1000.0, 1.0, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0), # or e >1.0
    (-10000.0, 1.0, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-100000.0, 1.0, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'EquPara_2'),

    # Inclined Hyperbolic Orbit varying a
    (-10.0, 1.3, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'IncHyp_a_1'),
    (-100.0, 1.3, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-1000.0, 1.3, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-10000.0, 1.3, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-100000.0, 1.3, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'IncHyp_a_2'),
    # Inclined Hyperbolic Orbit varying e
    (-100000.0, 1.1, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'IncHyp_e_1'),
    (-100000.0, 1.2, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-100000.0, 1.3, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-100000.0, 1.4, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-100000.0, 1.5, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'IncHyp_e_2'),

    # Equatorial Hyperbolic Orbit varying a
    (-10.0, 1.3, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'EquHyp_a_1'),
    (-100.0, 1.3, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-1000.0, 1.3, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-10000.0, 1.3, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-100000.0, 1.3, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'EquHyp_a_2'),
    # Equatorial Hyperbolic Orbit varying e
    (-100000.0, 1.1, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'EquHyp_e_1'),
    (-100000.0, 1.2, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-100000.0, 1.3, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-100000.0, 1.4, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 0),
    (-100000.0, 1.5, 0.0, 0.0, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15, 'EquHyp_e_2'),

    # # Approaching circular orbit
    # (100000.0, 0.000001, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15),

    # These don't work
    # (10000000.0, 1.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 347.8 * mc.D2R, 85.3 * mc.D2R, 0.3986004415E+15), # or e >1.0
    # (-10, 0.9, 33.3*mc.D2R, 48.2*mc.D2R, 347.8*mc.D2R, 85.3*mc.D2R, 0.3986004415E+15)
])

# provide a unique test method name, starting with test_
def test_orb_elem_convert(a, e, i, AN, AP, f, mu, name, DispPlot=False):
    """Module Unit Test"""
    # each test method requires a single assert method to be called
    [testResults, testMessage] = orbElem(a, e, i, AN, AP, f, mu, name, DispPlot)
    assert testResults < 1, testMessage

# Run unit test
def orbElem(a, e, i, AN, AP, f, mu, name, DispPlot):
    # Elem2RV
    testFailCount1 = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    # Create a sim module as an empty container
    unitTaskName = "unitTask"  # arbitrary name (don't change)
    unitProcessName = "TestProcess"  # arbitrary name (don't change)

    # Create a sim module as an empty container
    TotalSim = SimulationBaseClass.SimBaseClass()

    DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)
    # # create the dynamics task and specify the integration update time
    testProcessRate = macros.sec2nano(1.0)
    DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, testProcessRate))

    # Initialize the modules that we are using.
    orb_elemObject = orbElemConvert.OrbElemConvert()
    orb_elemObject.ModelTag = "OrbElemConvertData"

    # Add Model To Task
    TotalSim.AddModelToTask(unitTaskName, orb_elemObject)

    # Set element values
    epsDiff = 0.0000001
    orb_elemObject.mu = mu

    ###### ELEM2RV ######
    TotalSim.AddVariableForLogging('OrbElemConvertData.r_N', testProcessRate, 0, 2, 'double')
    TotalSim.AddVariableForLogging('OrbElemConvertData.v_N', testProcessRate, 0, 2, 'double')

    # Create and write messages
    ElemMessage = messaging.ClassicElementsMsgPayload()
    elemMsg = messaging.ClassicElementsMsg()

    if e == 1.0:
        ElemMessage.a = 0.0
        ElemMessage.rPeriap = -a
    else:
        ElemMessage.a = a # meters
    ElemMessage.e = e
    ElemMessage.i = i
    ElemMessage.Omega = AN
    ElemMessage.omega = AP
    ElemMessage.f = f

    elemMsg.write(ElemMessage)
    orb_elemObject.elemInMsg.subscribeTo(elemMsg)

    # Log Message to test WriteOutputMessage()
    dataLog = orb_elemObject.spiceStateOutMsg.recorder()
    TotalSim.AddModelToTask(unitTaskName, dataLog)

    # Execute simulation
    TotalSim.ConfigureStopTime(int(1E9))
    TotalSim.InitializeSimulation()
    TotalSim.ExecuteSimulation()

    # Get r and v from sim
    vSim = TotalSim.GetLogVariableData('OrbElemConvertData.v_N')
    vSim = numpy.delete(vSim[-1], 0, axis=0)
    rSim = TotalSim.GetLogVariableData('OrbElemConvertData.r_N')
    rSim = numpy.delete(rSim[-1], 0, axis=0)

    # Get r and v from message
    rMsgPlanet = dataLog.PositionVector[-1]
    vMsgPlanet = dataLog.VelocityVector[-1]
    rMsgPlanetDiff = numpy.subtract(rSim, rMsgPlanet)
    for g in range(3):
        if abs(rMsgPlanetDiff[g]) > 0:
            testMessages.append(" FAILED: Planet Position Message, column " + str(g))
            testFailCount1 += 1
    vMsgPlanetDiff = numpy.subtract(vSim, vMsgPlanet)
    for g in range(3):
        if abs(vMsgPlanetDiff[g]) > 0:
            testMessages.append(" FAILED: Planet Velocity Message, column " + str(g))
            testFailCount1 += 1

    # Calculation of elem2rv
    if e == 1.0 and a > 0.0:  # rectilinear elliptic orbit case
        Ecc = f  # f is treated as ecc.anomaly
        r = a * (1 - e * math.cos(Ecc))  # orbit radius
        v = math.sqrt(2 * mu / r - mu / a)
        ir = numpy.zeros(3)
        ir[0] = math.cos(AN) * math.cos(AP) - math.sin(AN) * math.sin(AP) * math.cos(i)
        ir[1] = math.sin(AN) * math.cos(AP) + math.cos(AN) * math.sin(AP) * math.cos(i)
        ir[2] = math.sin(AP) * math.sin(i)
        rTruth = numpy.multiply(r, ir)
        if math.sin(Ecc) > 0:
            vTruth = numpy.multiply(-v, ir)
        else:
            vTruth = numpy.multiply(v, ir)
    else:
        if e == 1 and a < 0:  # parabolic case
            rp = -a  # radius at periapses
            p = 2 * rp  # semi-latus rectum
            a = 0.0
        else:  # elliptic and hyperbolic cases
            p = a * (1 - e * e)  # semi-latus rectum

        r = p / (1 + e * math.cos(f))  # orbit radius
        theta = AP + f  # true latitude angle
        h = math.sqrt(mu * p)  # orbit ang.momentum mag.

        rTruth = numpy.zeros(3)
        rTruth[0] = r * (math.cos(AN) * math.cos(theta) - math.sin(AN) * math.sin(theta) * math.cos(i))
        rTruth[1] = r * (math.sin(AN) * math.cos(theta) + math.cos(AN) * math.sin(theta) * math.cos(i))
        rTruth[2] = r * (math.sin(theta) * math.sin(i))

        vTruth = numpy.zeros(3)
        vTruth[0] = -mu / h * (math.cos(AN) * (math.sin(theta) + e * math.sin(AP)) + math.sin(AN) * (math.cos(
            theta) + e * math.cos(AP)) * math.cos(i))
        vTruth[1] = -mu / h * (math.sin(AN) * (math.sin(theta) + e * math.sin(AP)) - math.cos(AN) * (math.cos(
            theta) + e * math.cos(AP)) * math.cos(i))
        vTruth[2] = -mu / h * (-(math.cos(theta) + e * math.cos(AP)) * math.sin(i))

    # Position and Velocity Diff Checks
    rDiff = numpy.subtract(rSim, rTruth)
    vDiff = numpy.subtract(vSim, vTruth)
    rDiffcsv = numpy.asarray(rDiff)
    vDiffcsv = numpy.asarray(vDiff)
    for g in range(3):
        if abs(rDiff[g]) > epsDiff:
            testMessages.append(" FAILED: Position Vector, column " + str(g))
            testFailCount1 += 1
    for g in range(3):
        if abs(vDiff[g]) > epsDiff:
            testMessages.append(" FAILED: Velocity Vector, column " + str(g))
            testFailCount1 += 1

    if name != 0:
        if testFailCount1 == 0:
            colorText = 'ForestGreen'
            passFailMsg = ""  # "Passed: " + name + "."
            passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
        else:
            colorText = 'Red'
            passFailMsg = "Failed: " + name + "."
            testMessages.append(passFailMsg)
            testMessages.append(" | ")
            passedText = r'\textcolor{' + colorText + '}{' + "FAILED" + '}'

        # Write some snippets for AutoTex
        snippetName = name + "PassedText1"
        snippetContent = passedText
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)

        snippetName = name + "PassFailMsg1"
        snippetContent = passFailMsg
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)

    ###### RV2ELEM ######
    # RV2Elem
    testFailCount2 = 0  # zero unit test result counter

    for g in range(2):
        TotalSim = SimulationBaseClass.SimBaseClass()
        DynUnitTestProc = TotalSim.CreateNewProcess(unitProcessName)

        # # create the dynamics task and specify the integration update time
        testProcessRate = macros.sec2nano(1.0)
        DynUnitTestProc.addTask(TotalSim.CreateNewTask(unitTaskName, testProcessRate))

        # Initialize the modules that we are using.
        orb_elemObject = orbElemConvert.OrbElemConvert()
        orb_elemObject.ModelTag = "OrbElemConvertData"

        # Add Model To Task
        TotalSim.AddModelToTask(unitTaskName, orb_elemObject)

        # Log Variables
        TotalSim.AddVariableForLogging('OrbElemConvertData.CurrentElem.a')
        TotalSim.AddVariableForLogging('OrbElemConvertData.CurrentElem.e')
        TotalSim.AddVariableForLogging('OrbElemConvertData.CurrentElem.i')
        TotalSim.AddVariableForLogging('OrbElemConvertData.CurrentElem.Omega')
        TotalSim.AddVariableForLogging('OrbElemConvertData.CurrentElem.omega')
        TotalSim.AddVariableForLogging('OrbElemConvertData.CurrentElem.f')
        TotalSim.AddVariableForLogging('OrbElemConvertData.r_N', testProcessRate, 0, 2, 'double')
        TotalSim.AddVariableForLogging('OrbElemConvertData.v_N', testProcessRate, 0, 2, 'double')

        orb_elemObject.mu = mu

        if g == 0:
            CartMessage = messaging.SCStatesMsgPayload()
            CartMessage.r_BN_N = rSim
            CartMessage.v_BN_N = vSim
            stateScMsg = messaging.SCStatesMsg().write(CartMessage)
            orb_elemObject.scStateInMsg.subscribeTo(stateScMsg)
        else:
            CartMessage = messaging.SpicePlanetStateMsgPayload()
            CartMessage.PositionVector = rSim
            CartMessage.VelocityVector = vSim
            stateSpMsg = messaging.SpicePlanetStateMsg().write(CartMessage)
            orb_elemObject.spiceStateInMsg.subscribeTo(stateSpMsg)

        dataElemLog = orb_elemObject.elemOutMsg.recorder()
        TotalSim.AddModelToTask(unitTaskName, dataElemLog)

        # Execute simulation
        TotalSim.ConfigureStopTime(int(1E9))
        TotalSim.InitializeSimulation()
        TotalSim.ExecuteSimulation()

        aOut = TotalSim.GetLogVariableData('OrbElemConvertData.CurrentElem.a')
        aOut = numpy.delete(aOut[-1], 0, axis=0)
        eOut = TotalSim.GetLogVariableData('OrbElemConvertData.CurrentElem.e')
        eOut = numpy.delete(eOut[-1], 0, axis=0)
        iOut = TotalSim.GetLogVariableData('OrbElemConvertData.CurrentElem.i')
        iOut = numpy.delete(iOut[-1], 0, axis=0)
        ANOut = TotalSim.GetLogVariableData('OrbElemConvertData.CurrentElem.Omega')
        ANOut = numpy.delete(ANOut[-1], 0, axis=0)
        APOut = TotalSim.GetLogVariableData('OrbElemConvertData.CurrentElem.omega')
        APOut = numpy.delete(APOut[-1], 0, axis=0)
        fOut = TotalSim.GetLogVariableData('OrbElemConvertData.CurrentElem.f')
        fOut = numpy.delete(fOut[-1], 0, axis=0)

        # Element Diff Check
        ElemDiff = [(a - aOut), (e - eOut), (i - iOut), (AN - ANOut), (AP - APOut), (f - fOut)]
        ElemDiffcsv = numpy.asarray(ElemDiff)
        for g in range(6):
            # check for angle roll over with 2*pi
            if g > 2:
                if abs(ElemDiff[g] - 2 * math.pi) < epsDiff:
                    ElemDiff[g] -= 2 * math.pi
                elif abs(ElemDiff[g] + 2 * math.pi) < epsDiff:
                    ElemDiff[g] += 2 * math.pi
            if abs(ElemDiff[g]) > epsDiff:
                testMessages.append(" FAILED: Sim Orbital Element " + str(g))
                testFailCount2 += 1

    aMsg = dataElemLog.a[-1]
    eMsg = dataElemLog.e[-1]
    iMsg = dataElemLog.i[-1]
    ANMsg = dataElemLog.Omega[-1]
    APMsg = dataElemLog.omega[-1]
    fMsg = dataElemLog.f[-1]

    ElemMsgDiff = [(aOut - aMsg), (eOut - eMsg), (iOut - iMsg), (ANOut - ANMsg), (APOut - APMsg), (fOut - fMsg)]
    for g in range(6):
        # check for angle roll over with 2*pi
        if g > 2:
            if abs(ElemDiff[g] - 2 * math.pi) < epsDiff:
                ElemDiff[g] -= 2 * math.pi
            elif abs(ElemDiff[g] + 2 * math.pi) < epsDiff:
                ElemDiff[g] += 2 * math.pi
        if abs(ElemMsgDiff[g]) > 0:
            testMessages.append(" FAILED: Orbital Element Message " + str(g))
            testFailCount2 += 1
    ######### Calculate rv2elem #########
    # Calculate the specific angular momentum and its magnitude
    epsConv = 0.0000000001
    hVec = numpy.cross(rTruth, vTruth)
    h = numpy.linalg.norm(hVec)
    p = h * h / mu

    # Calculate the line of nodes
    v3 = numpy.array([0.0, 0.0, 1.0])
    nVec = numpy.cross(v3, hVec)
    n = numpy.linalg.norm(nVec)

    # Orbit eccentricity and energy
    r = numpy.linalg.norm(rTruth)
    v = numpy.linalg.norm(vTruth)
    eVec = numpy.multiply(v * v / mu - 1.0 / r, rTruth)
    v3 = numpy.multiply(numpy.dot(rTruth, vTruth) / mu, vTruth)
    eVec = numpy.subtract(eVec, v3)
    eO = numpy.linalg.norm(eVec)
    rmag = r
    rPeriap = p / (1.0 + eO)

    # compute semi - major axis
    alpha = 2.0 / r - v * v / mu
    if (math.fabs(alpha) > epsConv): # elliptic or hyperbolic case
        aO = 1.0 / alpha
        rApoap = p / (1.0 - eO)
    else:                        # parabolic case
        rp = p / 2.0
        aO = 0.0 # a is not defined for parabola, so -rp is returned instead
        rApoap = 0.0

    # Calculate the inclination
    iO = math.acos(hVec[2] / h)

    # Calculate AP, AN, and True anomaly
    if eO >= 1e-11 and iO >= 1e-11:
    # Case 1: Non - circular, inclined orbit
        Omega = math.acos(nVec[0] / n)
        if (nVec[1] < 0.0):
            Omega = 2.0 * math.pi - Omega
        omega = math.acos(numpy.dot(nVec, eVec) / n / eO)
        if eVec[2] < 0.0:
            omega = 2.0 * math.pi - omega
        fO = math.acos(numpy.dot(eVec, rTruth) / eO / r)
        if numpy.dot(rTruth, vTruth) < 0.0:
            fO = 2.0 * math.pi - fO
    elif eO >= 1e-11 and iO < 1e-11:
    # Case 2: Non - circular, equatorial orbit
    # Equatorial orbit has no ascending node
        Omega = 0.0
        # True longitude of periapsis, omegatilde_true
        omega = math.acos(eVec[0] / eO)
        if eVec[1] < 0.0:
            omega = 2.0 * math.pi - omega
        fO = math.acos(numpy.dot(eVec, rTruth) / eO / r)
        if numpy.dot(rTruth, vTruth) < 0.0:
            fO = 2.0 * math.pi - fO
    elif eO < 1e-11 and iO >= 1e-11:
    # Case 3: Circular, inclined orbit
        Omega = math.acos(nVec[0] / n)
        if (nVec[1] < 0.0):
            Omega = 2.0 * math.pi - Omega
        omega = 0.0
        # Argument of latitude, u = omega + f * /
        fO = math.acos(numpy.dot(nVec, rTruth) / n / r)
        if rTruth[2] < 0.0:
            fO = 2.0 * math.pi - fO
    elif eO < 1e-11 and iO < 1e-11:
    # Case 4: Circular, equatorial orbit
        Omega = 0.0
        omega = 0.0
        # True longitude, lambda_true
        fO = math.acos(rTruth[0] / r)
        if rTruth[1] < 0:
            fO = 2.0 * math.pi - fO
    else:
        print("Error: rv2elem couldn't identify orbit type.\n")
    if (eO >= 1.0 and math.fabs(fO) > math.pi):
        twopiSigned = math.copysign(2.0 * math.pi, fO)
        fO -= twopiSigned

    # Element Diff Check
    ElemCalcDiff = [(aO - aOut), (eO - eOut), (iO - iOut), (Omega - ANOut), (omega - APOut), (fOut - fOut)]
    ElemCalcDiffcsv = numpy.asarray(ElemCalcDiff)
    for g in range(6):
        # check for angle roll over with 2*pi
        if g > 2:
            if abs(ElemCalcDiff[g] - 2 * math.pi) < epsDiff:
                ElemCalcDiff[g] -= 2 * math.pi
            elif abs(ElemCalcDiff[g] + 2 * math.pi) < epsDiff:
                ElemCalcDiff[g] += 2 * math.pi
        if abs(ElemCalcDiff[g]) > epsDiff:
            testMessages.append(" FAILED: Calculated Orbital Element " + str() + str(g))
            testFailCount2 += 1

    # create plot
    # txt = 'e = ' + str(e) + ' and a = ' + str(a) + 'km'
    fact = (len(str(abs(a)))-3.0)

    plt.figure(1,figsize=(7, 5), dpi=80, facecolor='w', edgecolor='k')
    plt.clf()
    # fig1.text(.5, .05, txt, ha='center')
    ax1 = plt.subplot(211)
    ax1.cla()
    index = numpy.arange(3)
    bar_width = 0.35
    opacity = 0.8
    rects1 = ax1.bar(index, rSim, bar_width, alpha=opacity, color='b', label='Simulated Position')
    rects2 = ax1.bar(index + bar_width, rTruth, bar_width, alpha=opacity, color='g', label='Calculated Position')
    ax1.spines['left'].set_position('zero')
    ax1.spines['right'].set_color('none')
    ax1.spines['bottom'].set_position('zero')
    ax1.spines['top'].set_color('none')
    for xtick in ax1.get_xticklabels():
        xtick.set_bbox(dict(facecolor='white', edgecolor='None', alpha=0.5))
    ax1.xaxis.set_ticks_position('bottom')
    ax1.yaxis.set_ticks_position('left')
    plt.ylabel('Position (m)')
    plt.xticks(index + bar_width, ('X', 'Y', 'Z'))
    plt.legend(loc='lower right')

    ax2 = plt.subplot(212)
    ax2.cla()
    rects1 = ax2.bar(index, vSim, bar_width, alpha=opacity, color='b', label='Simulated Velocity')
    rects2 = ax2.bar(index + bar_width, vTruth, bar_width, alpha=opacity, color='g', label='Calculated Velocity')
    ax2.spines['left'].set_position('zero')
    ax2.spines['right'].set_color('none')
    ax2.spines['bottom'].set_position('zero')
    ax2.spines['top'].set_color('none')
    for xtick in ax2.get_xticklabels():
        xtick.set_bbox(dict(facecolor='white', edgecolor='None', alpha=0.5))
    ax2.xaxis.set_ticks_position('bottom')
    ax2.yaxis.set_ticks_position('left')
    plt.ylabel('Velocity (m/s)')
    plt.xticks(index + bar_width, ('X', 'Y', 'Z'))
    plt.legend(loc='lower right')

    if name != 0:
        unitTestSupport.writeFigureLaTeX(name, "$e = " + str(e) + "$ and $a = 10^" + str(int(fact)) + "$km",
                                         plt, 'height=0.7\\textwidth, keepaspectratio', path)
        if testFailCount2 == 0:
            colorText = 'ForestGreen'
            passFailMsg = ""  # "Passed: " + name + "."
            passedText = r'\textcolor{' + colorText + '}{' + "PASSED" + '}'
        else:
            colorText = 'Red'
            passFailMsg = "Failed: " + name + "."
            testMessages.append(passFailMsg)
            testMessages.append(" | ")
            passedText = r'\textcolor{' + colorText + '}{' + "FAILED" + '}'

        # Write some snippets for AutoTex
        snippetName = name + "PassedText2"
        snippetContent = passedText
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)

        snippetName = name + "PassFailMsg2"
        snippetContent = passFailMsg
        unitTestSupport.writeTeXSnippet(snippetName, snippetContent, path)

    if DispPlot:
        plt.show()
        plt.close()
    testFailCount = testFailCount1+testFailCount2

    if testFailCount:
        print("Failed")
        print(testMessages)
    else:
        print("PASSED")

    return [testFailCount, ''.join(testMessages)]

if __name__ == "__main__":
    orbElem(10000000.0, 0.0, 33.3 * mc.D2R, 48.2 * mc.D2R, 0.0, 85.3 * mc.D2R, 0.3986004415E+15, 0, True
                          )
