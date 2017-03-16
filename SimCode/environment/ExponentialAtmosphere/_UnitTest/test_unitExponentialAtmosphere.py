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
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Test the validity of a simple exponential atmosphere model.
# Author:   Andrew Harris
# Creation Date:  Jan 18, 2017
#

import sys, os, inspect
import matplotlib
import numpy as np
import ctypes
import math
import csv
import logging
import pytest


# @cond DOXYGEN_IGNORE
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))
bskName = 'Basilisk'
splitPath = path.split(bskName)
bskPath = splitPath[0] + '/' + bskName + '/'
# if this script is run from a custom folder outside of the Basilisk folder, then uncomment the
# following line and specify the absolute bath to the Basilisk folder
#bskPath = '/Users/hp/Documents/Research/' + bskName + '/'
sys.path.append(bskPath + 'modules')
sys.path.append(bskPath + 'PythonModules')
# @endcond

# import general simulation support files
import SimulationBaseClass
import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
import macros
import orbitalMotion

# import simulation related support
import spacecraftPlus
import gravityEffector
import simIncludeGravity
import exponentialAtmosphere
import dragDynamicEffector
import unitTestSupport
#print dir(exponentialAtmosphere)


def test_unitAtmosphere():
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    #   Initialize new atmosphere and drag model, add them to task
    newAtmo = exponentialAtmosphere.ExponentialAtmosphere()
    atmoTaskName = "atmosphere"
    newAtmo.ModelTag = "ExpAtmo"

    showVal = False
    testResults = []
    testMessage = []

    planetRes, planetMsg = test_setPlanet(newAtmo)
    testMessage.append(planetMsg)
    testResults.append(planetRes)
    test_setBaseDens(newAtmo)

    planetRes, planetMsg = test_setScaleHeight(newAtmo)
    testMessage.append(planetMsg)
    testResults.append(planetRes)
    test_setBaseDens(newAtmo)

    planetRes, planetMsg = test_setPlanetRadius(newAtmo)
    testMessage.append(planetMsg)
    testResults.append(planetRes)
    test_setBaseDens(newAtmo)

    planetRes, planetMsg = test_setPlanet(newAtmo)
    testMessage.append(planetMsg)
    testResults.append(planetRes)
    test_setBaseDens(newAtmo)

    planetRes, planetMsg = test_AddSpacecraftToModel(newAtmo)
    testMessage.append(planetMsg)
    testResults.append(planetRes)
    test_setBaseDens(newAtmo)

    testSum = sum(testResults)
    assert testSum < 1, testMessage

def test_setPlanet(atmoModel):
    testFailCount = 0
    testMessages = []
    nameVec = ["Venus","Earth","Mars","Krypton"]
    for name in nameVec:
        atmoModel.SetPlanet(name)
        if atmoModel.planetName != name:
            testFailCount += 1
            testMessages.append(
                "FAILED: ExponentialAtmosphere could not set planet name to "+name+".")
    return testFailCount, testMessages

def test_setBaseDens(atmoModel):
    testFailCount = 0
    testMessages = []
    testDens = 10.0
    atmoModel.SetBaseDensity(testDens)
    if atmoModel.atmosphereProps.baseDensity != testDens:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere could not set base density value.")
    return testFailCount, testMessages

def test_setScaleHeight(atmoModel):
    testFailCount = 0
    testMessages = []
    testScaleHeight = 20000.0
    atmoModel.SetScaleHeight(testScaleHeight)
    if atmoModel.atmosphereProps.scaleHeight != testScaleHeight:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere could not set scale height.")
    return testFailCount, testMessages

def test_setPlanetRadius(atmoModel):
    testFailCount = 0
    testMessages = []
    testPlanetRadius = 7000.0*1000.0
    atmoModel.SetPlanetRadius(testPlanetRadius)
    if atmoModel.atmosphereProps.planetRadius != testPlanetRadius:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere could not set planet radius.")
    return testFailCount, testMessages

def test_AddSpacecraftToModel(atmoModel):
    testFailCount = 0
    testMessages = []

    # create the dynamics task and specify the integration update time
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.useTranslation = True
    scObject.hub.useRotation = False

    scObject2 = spacecraftPlus.SpacecraftPlus()
    scObject2.ModelTag = "spacecraftBody"
    scObject2.hub.useTranslation = True
    scObject2.hub.useRotation = False

    # add spacecraftPlus object to the simulation process
    atmoModel.AddSpacecraftToModel(scObject.scStateOutMsgName)
    atmoModel.AddSpacecraftToModel(scObject2.scStateOutMsgName)

    if len(atmoModel.scStateInMsgNames) != 2:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere does not have enough input message names.")

    if len(atmoModel.atmoDensOutMsgNames) != 2:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere does not have enough output message names.")
    return testFailCount, testMessages



