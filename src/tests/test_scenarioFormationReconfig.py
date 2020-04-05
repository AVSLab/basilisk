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

import pytest
import os
import sys
import inspect
from Basilisk.utilities import unitTestSupport

# Get current file path
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../examples')
import scenarioFormationReconfig


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useRefAttitude", [(True), (False)])
@pytest.mark.scenarioTest

# provide a unique test method name, starting with test_
def test_scenarioFormationReconfig(show_plots, useRefAttitude):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataPos, dataVel, dataPos2, dataVel2, dataAttErr, numDataPoints, figureList = \
        scenarioFormationReconfig.run(show_plots, useRefAttitude)

    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataPos = dataPos[::skipValue]
    dataVel = dataVel[::skipValue]
    dataPos2 = dataPos2[::skipValue]
    dataVel2 = dataVel2[::skipValue]
    dataAttErr = dataAttErr[::skipValue]

    # setup truth data for unit test
    truePos = [
        [-1698168.4591564279981, -4442389.7052656793967, 2941314.0510698854923]
        , [2554228.1187763377093, -7619533.2694213027135, -4424052.8758416855708]
        , [4632673.2753893705085, -2369007.5104981116019, -8024025.4878409523517]
        , [2454529.0588506101631, 4524015.2429778426886, -4251369.0385834760964]
        , [-2128934.0162037736736, -2390867.8602575766854, 3687421.8820266118273]
    ]
    trueVel = [
        [2447.0772820860888714, -7852.8023960875389093, -4238.4621826206630431]
        , [2595.9975372950757446, 1732.7985059992488459, -4496.3996309187787119]
        , [196.43180259520977415, 5124.6661363806988447, -340.22986231723018591]
        , [-3613.1515986700246685, 3202.3655760100173211, 6258.1621443452086169]
        , [1192.2419013702201482, -10031.19836724428751, -2065.0235480857654693]
    ]
    truePos2 = trueVel2 = []
    if(useRefAttitude):
        truePos2 = [
            [-1697505.0733689018525,-4444016.7098212577403,2943923.5337480967864]
            , [2555059.6817299821414,-7621914.0329712536186,-4421233.1954132523388]
            , [4632835.1045218557119,-2373234.380998193752,-8024508.8480127304792]
            , [2456276.8041306459345,4520958.1577555546537,-4255596.0724062742665]
            , [-2128654.6843600063585,-2387483.9660659534857,3688233.4080739826895]
        ]
        trueVel2 = [
            [2448.72017978365011,-7850.2509187729565383,-4236.8470137883041389]
            , [2595.6976025823496457,1731.392110967623239,-4498.1180485950426373]
            , [195.79525407079719912,5123.5736191893456635,-342.74691367460900437]
            , [-3610.0995145898345982,3206.5205593700229656,6256.2853629327628369]
            , [1188.2233444639464324,-10034.531334298460933,-2060.6861945262844529]
        ]
    else:
        truePos2 = [
            [-1697505.0733689018525, -4444016.7098212577403, 2943923.5337480967864]
            , [2555059.6817299816757, -7621914.0329712564126, -4421233.1954132523388]
            , [4632833.4792611878365, -2373238.5519547993317, -8024511.1737792296335]
            , [2456273.90102655394, 4520941.7709022331983, -4255603.7321277642623]
            , [-2128620.7312902896665, -2387568.5335029629059, 3688183.146928451024]
        ]
        trueVel2 = [
            [2448.72017978365011, -7850.2509187729565383, -4236.8470137883041389]
            , [2595.6976025823491909, 1731.3921109676200558, -4498.1180485950426373]
            , [195.79260074038586481, 5123.5659350719815848, -342.75156612422159697]
            , [-3610.1002022358325121, 3206.5114480741831358, 6256.2882556858839962]
            , [1188.310644178294524, -10034.505007102014133, -2060.8221775392112249]
        ]
    trueAttErr = []
    if(useRefAttitude):
        trueAttErr = [
            [-1,0,0]
            , [2.6107360581412042381e-09,0,0]
            , [0.00019598169404489229682,0.00017266742139225124447,-0.00012919605688021837766]
            , [0.00011065463440602744114,1.9841077177434126596e-05,1.3249651829369002307e-05]
            , [0.00013428561852345789426,0.00034508490987785291667,0.00058406533494748862165]
        ]
    else:
        trueAttErr = [
            [-0.168285299854870396, 0.34884381524610280634, 0.42358637266618048844]
            , [4.5113159741014079012e-10, -1.3659255979800949449e-09, -1.0447446842249802979e-09]
            , [-7.8440956385435176338e-05, 0.00020382917829231828998, 6.9350429093225312129e-05]
            , [7.5574859807112218296e-05, 4.5699994109496602779e-05, -3.4534300622735931649e-05]
            , [6.7117487342204932522e-13, 3.8425764073158065845e-13, -2.581395081005749554e-13]
        ]

    # compare the results to the truth values
    accuracy = 1e-6

    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos, dataPos, accuracy, "chief r_BN_N Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueVel, dataVel, accuracy, "chief v_BN_N Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArray(
        truePos2, dataPos2, accuracy, "deputy r_BN_N Vector",
        testFailCount, testMessages)

    testFailCount, testMessages = unitTestSupport.compareArray(
        trueVel2, dataVel2, accuracy, "deputy v_BN_N Vector",
        testFailCount, testMessages)
    
    testFailCount, testMessages = unitTestSupport.compareArray(
        trueAttErr, dataAttErr, accuracy, "deputy attitude Error",
        testFailCount, testMessages)

    # save the figures to the Doxygen scenario images folder
    for pltName, plt in list(figureList.items()):
        unitTestSupport.saveScenarioFigure(pltName, plt, path)

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print("# Errors:", testFailCount)
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found
    assert testFailCount < 1, testMessages
