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
import scenarioFormationMeanOEFeedback


# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True)

# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("useClassicElem", [(True), (False)])
@pytest.mark.scenarioTest

# provide a unique test method name, starting with test_
def test_bskFormationMeanOEFeedback(show_plots, useClassicElem):
    '''This function is called by the py.test environment.'''
    # each test method requires a single assert method to be called

    testFailCount = 0  # zero unit test result counter
    testMessages = []  # create empty array to store test log messages

    dataPos, dataVel, dataPos2, dataVel2, numDataPoints, figureList = \
        scenarioFormationMeanOEFeedback.run(show_plots, useClassicElem)

    numTruthPoints = 5
    skipValue = int(numDataPoints / numTruthPoints)
    dataPos = dataPos[::skipValue]
    dataVel = dataVel[::skipValue]
    dataPos2 = dataPos2[::skipValue]
    dataVel2 = dataVel2[::skipValue]

    # setup truth data for unit test
    truePos = [
        [1436484.60196780902333557605743, 3886749.62927295779809355735779, 685338.826899446081370115280151]
        , [-5098330.72509963158518075942993, 2155672.96920512244105339050293, 289352.591602899658028036355972]
        , [-6849381.35642359219491481781006, -2991503.07133573386818170547485, -751830.03624052763916552066803]
        , [-5055828.8957376871258020401001, -7195142.31145382393151521682739, -1471792.19251434411853551864624]
        , [-1513479.99552857177332043647766, -9373574.18936761654913425445557, -1633504.36709573515690863132477]
    ]
    trueVel = [
        [-10831.63649328396786586381495, 3882.49945963648633551201783121, 684.589407319948463737091515213]
        , [-6400.68993446120566659374162555, -6645.26742884397026500664651394, -1280.70961251381640977342613041]
        , [146.133746114545033378817606717, -6896.99022613049692154163494706, -1190.41866478448378074972424656]
        , [3517.18326440737428129068575799, -4424.67730609357295179506763816, -571.382712472215871457592584193]
        , [4858.56690333148708305088803172, -1410.61371727026653388747945428, 91.1417787223904980464794789441]
    ]
    truePos2 = trueVel2 = []
    if(useClassicElem):
        truePos2 = [
            [1426591.04298178106546401977539, 3889479.96808567875996232032776, 684516.703467827639542520046234]
            , [-5102385.66772917564958333969116, 2149405.55787777854129672050476, 288538.68037558230571448802948]
            , [-6847769.60182628408074378967285, -2997068.04093963187187910079956, -752364.675480989622883498668671]
            , [-5051654.91263425163924694061279, -7198523.10098007787019014358521, -1471996.82998917996883392333984]
            , [-1508236.10134511487558484077454, -9374580.59191515482962131500244, -1633436.6776368594728410243988]
        ]
        trueVel2 = [
            [-10841.4770222591942001599818468, 3862.11833323184964683605358005, 680.564331324959084668080322444]
            , [-6391.75446832767011073883622885, -6651.66255377470497478498145938, -1281.22220131563153699971735477]
            , [151.22522761920967582227603998, -6896.24576672574312397046014667, -1190.19634418304008249833714217]
            , [3519.73296114427148495451547205, -4422.31261392575834179297089577, -571.108333447736868038191460073]
            , [4859.28977315332940634107217193, -1407.75439236288343636260833591, 91.3877037121594639756949618459]
        ]
    else:
        truePos2 = [
            [1426591.04298178106546401977539, 3889479.96808567875996232032776, 684516.703467827639542520046234]
            , [-5098193.2236329382285475730896, 2155223.04628036124631762504578, 293085.629591327859088778495789]
            , [-6849682.12169486284255981445312, -2992176.80240494525060057640076, -749193.974887459655292332172394]
            , [-5055937.15921268146485090255737, -7195228.73048102948814630508423, -1471954.30587741150520741939545]
            , [-1513418.57660906319506466388702, -9373100.61301772110164165496826, -1636601.80398252932354807853699]
        ]
        trueVel2 = [
            [-10841.4770222591942001599818468, 3862.11833323184964683605358005, 680.564331324959084668080322444]
            , [-6400.9617667769780382513999939, -6645.43118169308945653028786182, -1279.43701916828786124824546278]
            , [146.394284025049842057342175394, -6896.20662860978609387530013919, -1193.33268764812942208664026111]
            , [3517.37173313155881260172463953, -4423.84925452640254661673679948, -575.242033860933702271722722799]
            , [4858.76680106058483943343162537, -1409.9707687166030609660083428, 87.7666731044913888126757228747]
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
