''' '''
'''
 ISC License

 Copyright (c) 2016, Autonomous Vehicle Systems Lab, University of Colorado at Boulder

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



# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import unitTestSupport                  # general support file with common unit test functions
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# import simulation related support
from Basilisk.simulation import spacecraftPlus
from Basilisk.simulation import exponentialAtmosphere
from Basilisk.simulation import dragDynamicEffector
from Basilisk.utilities import unitTestSupport
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

    planetRes, planetMsg = setPlanet(newAtmo)
    testMessage.append(planetMsg)
    testResults.append(planetRes)

    planetRes, planetMsg = AddSpacecraftToModel(newAtmo)
    testMessage.append(planetMsg)
    testResults.append(planetRes)

    testSum = sum(testResults)
    assert testSum < 1, testMessage

def setPlanet(atmoModel):
    testFailCount = 0
    testMessages = []
    nameVec = ["venus","earth","mars","krypton"]
    for name in nameVec:
        atmoModel.setPlanet(name)
        if atmoModel.planetName != name:
            testFailCount += 1
            testMessages.append(
                "FAILED: ExponentialAtmosphere could not set planet name to "+name+".")
    return testFailCount, testMessages

def AddSpacecraftToModel(atmoModel):
    testFailCount = 0
    testMessages = []

    # create the dynamics task and specify the integration update time
    scObject = spacecraftPlus.SpacecraftPlus()
    scObject.ModelTag = "spacecraftBody"

    scObject2 = spacecraftPlus.SpacecraftPlus()
    scObject2.ModelTag = "spacecraftBody"
    # add spacecraftPlus object to the simulation process
    atmoModel.addSpacecraftToModel(scObject.scStateOutMsgName)
    atmoModel.addSpacecraftToModel(scObject2.scStateOutMsgName)

    if len(atmoModel.scStateInMsgNames) != 2:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere does not have enough input message names.")

    if len(atmoModel.atmoDensOutMsgNames) != 2:
        testFailCount += 1
        testMessages.append(
            "FAILED: ExponentialAtmosphere does not have enough output message names.")
    return testFailCount, testMessages



