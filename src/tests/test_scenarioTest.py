#
#  ISC License
#
#  Copyright (c) 2021, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#


#
# Integrated tests
#
# Purpose:  This script calls a series of basic scenario tutorials to ensure
# that they complete properly.  No simulation values are returned and tested.
# Author:   Hanspeter Schaub
# Creation Date:  May 20, 2021
#


import importlib
import inspect
import os
import sys

import pytest
from Basilisk.utilities import simHelpers

filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

sys.path.append(path + '/../../examples/')

# uncomment this line is this test is to be skipped in the global unit test run, adjust message as needed
# @pytest.mark.skipif(conditionstring)
# uncomment this line if this test has an expected failure, adjust message as needed
# @pytest.mark.xfail(True, reason="Previously set sim parameters are not consistent with new formulation\n")


# The following 'parametrize' function decorator provides the parameters and expected results for each
#   of the multiple test runs for this test.
@pytest.mark.parametrize("scenarioCase", [
                                        'scenarioGroundLocationImaging'
                                        , 'scenarioGroundDownlink'
                                        , 'scenarioMtbMomentumManagement'
                                        , 'scenarioMtbMomentumManagementSimple'
                                        , 'scenarioSmallBodyNav'
                                        , 'scenarioSmallBodyNavUKF'
                                        , 'scenarioSmallBodyFeedbackControl'
                                        , 'scenarioJupiterArrival'
                                        , 'scenarioSpiceSpacecraft'
                                        , 'scenarioInertialSpiral'
                                        , 'scenarioDeployingPanel'
                                        , 'scenarioAsteroidArrival'
                                        , 'scenarioTwoChargedSC'
                                        , 'scenarioRendezVous'
                                        , 'scenarioSensorThermal'
                                        , 'scenarioHaloOrbit'
                                        , 'scenarioDeployingSolarArrays'
                                        , 'scenarioTempMeasurementAttitude'
                                        , 'scenarioRoboticArm'
                                        , 'scenarioOrbitManeuverTH'
                                        , 'scenarioLambertSolver'
                                        , 'scenarioQuadMaps'
                                        , 'scenarioExtendingBoom'
                                        , 'scenarioPrescribedMotionWithRotationBranching'
                                        , 'scenarioPrescribedMotionWithTranslationBranching'
                                        , 'scenarioImpact'
                                        , 'scenarioPrescribedScrewMotion'
                                        , 'scenarioGroundTracks'
                                        , 'scenarioStripImaging'
                                        , 'scenarioRotTransArrayDeployment'
                                        ])
@pytest.mark.scenarioTest
def test_scenarioBskScenarios(show_plots, scenarioCase):

    testFailCount = 0                       # zero unit test result counter
    testMessages = []                       # create empty array to store test log messages
    # import the bskSim script to be tested
    scene_plt = importlib.import_module(scenarioCase)

    try:
        figureList = scene_plt.run(False)

        # save the figures to the Doxygen scenario images folder

        if figureList != {}:
            for pltName, plt in list(figureList.items()):
                simHelpers.saveScenarioFigure(pltName, plt, path)

    except OSError as err:
        testFailCount = testFailCount + 1
        testMessages.append("OS error: {0}".format(err))

    #   print out success message if no error were found
    if testFailCount == 0:
        print("PASSED ")
    else:
        print("Failed: testFailCount is " + str(testFailCount))
        print(testMessages)

    # each test method requires a single assert method to be called
    # this check below just makes sure no sub-test failures were found

    assert testFailCount < 1, testMessages


@pytest.mark.scenarioTest
def test_solar_array_first_deployment_step():
    """The hub reacts to the first array's deployment during the first dynamics interval."""
    import scenarioDeployingSolarArrays

    figures = scenarioDeployingSolarArrays.run(False)
    array_rate = figures["scenarioDeployingSolarArrays_Array1ElementThetaDot"].axes[0].lines[0]
    hub_y_rate = figures["scenarioDeployingSolarArrays_HubInertialAngularVelocity"].axes[0].lines[1]
    assert array_rate.get_xdata()[1] == hub_y_rate.get_xdata()[1]
    assert array_rate.get_ydata()[0] == hub_y_rate.get_ydata()[0] == 0.0  # [deg/s]
    assert array_rate.get_ydata()[1] > 0.0 > hub_y_rate.get_ydata()[1]  # [deg/s]


@pytest.mark.scenarioTest
@pytest.mark.parametrize("scenario, step", [("scenarioPrescribedScrewMotion", 10_000_000),
                                            ("scenarioRotTransArrayDeployment", 50_000_000)])  # [ns]
def test_prescribed_profilers_publish_before_effectors(monkeypatch, scenario, step):
    """Check each effector's translation and rotation input headers when it runs."""
    from Basilisk.simulation.prescribedMotionStateEffector import PrescribedMotionStateEffector
    from Basilisk.utilities.SimulationBaseClass import SimBaseClass
    from Basilisk.utilities.pythonVariableLogger import PythonVariableLogger

    probes = []
    add_model = SimBaseClass.AddModelToTask
    execute = SimBaseClass.ExecuteSimulation

    def add_with_probe(sim, task, model, *args, **kwargs):
        if isinstance(model, PrescribedMotionStateEffector):
            inputs = (model.prescribedTranslationInMsg, model.prescribedRotationInMsg)
            probe = PythonVariableLogger({
                "commands": lambda tick: [reader.timeWritten() if reader.isWritten() else -1 for reader in inputs]
            })
            # Equal priority and adjacent insertion put the probe immediately before this effector.
            add_model(sim, task, probe, *args, **kwargs)
            probes.append(probe)
        return add_model(sim, task, model, *args, **kwargs)

    class FirstStepComplete(Exception):
        pass

    def run_first_step(sim):
        sim.ConfigureStopTime(step)
        execute(sim)
        raise FirstStepComplete

    monkeypatch.setattr(SimBaseClass, "AddModelToTask", add_with_probe)
    monkeypatch.setattr(SimBaseClass, "ExecuteSimulation", run_first_step)
    with pytest.raises(FirstStepComplete):
        importlib.import_module(scenario).run(False)

    assert probes
    for probe in probes:
        assert probe.times().tolist() == [0, step]
        assert probe.commands.tolist() == [[0, 0], [step, step]]
