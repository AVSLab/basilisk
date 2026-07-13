#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

# Runs every dynamics-engine comparison scenario in examples/dynamicsComparison,
# confirming each runs without error and saving its result figures.

import os
import sys
import importlib
import json
import tempfile

import numpy as np
import pytest
from Basilisk import hasBuildFeature
from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import RigidBodyKinematics as rbk

THIS_FOLDER = os.path.dirname(__file__)
SCENARIO_FOLDER = os.path.join(
    THIS_FOLDER, "..", "..", "examples", "dynamicsComparison"
)
SCENARIO_FILES = [
    scenarioFile[:-3]
    for scenarioFile in os.listdir(SCENARIO_FOLDER)
    if scenarioFile.startswith("scenario") and scenarioFile.endswith(".py")
]

# Per-scenario test overrides: reduced sweeps (and, for the Pareto studies, a cheaper
# reference and shorter horizon) to keep each test under the scenario runtime budget.
SCENARIO_RUN_KWARGS = {
    "scenarioCompareFlexPanels": {"nSegmentsList": (1, 2, 4, 8)},
    "scenarioCompareParetoRwPanels": {
        "sweepConfigs": [
            ("svIntegratorRK4", 0.1, None),
            ("svIntegratorEuler", 0.02, None),
            ("svIntegratorRKF45", 0.5, 1.0e-7),
        ],
        "reference": {"integrator": "svIntegratorRKF45", "dt": 0.05, "tol": 1.0e-9},
        "referenceCheck": {"integrator": "svIntegratorRKF45", "dt": 0.025, "tol": 1.0e-9},
    },
    "scenarioCompareParetoFlexPanels": {
        "sweepConfigs": [
            ("svIntegratorRKF45", 0.1, 1.0e-4),
            ("svIntegratorRKF45", 0.1, 1.0e-6),
        ],
        "simDuration": 1.0,
        "reference": {"integrator": "svIntegratorRKF45", "dt": 0.2, "tol": 1.0e-6},
    },
}

SCENARIO_EXPECTED_FIGURES = {
    "scenarioCompareFlexPanels": {
        "scenarioCompareFlexPanels_validity",
        "scenarioCompareFlexPanels_runtime",
    },
    "scenarioCompareOrbit": {
        "scenarioCompareOrbit_trajectory",
        "scenarioCompareOrbit_accuracy",
        "scenarioCompareOrbit_crossError",
    },
    "scenarioCompareOrbitMultibody": {
        "scenarioCompareOrbitMultibody_compactOrbit",
        "scenarioCompareOrbitMultibody_attError",
        "scenarioCompareOrbitMultibody_regimes",
    },
    "scenarioCompareParetoFlexPanels": {
        "scenarioCompareParetoFlexPanels_pareto",
        "scenarioCompareParetoFlexPanels_frontier",
        "scenarioCompareParetoFlexPanels_paretoPosition",
        "scenarioCompareParetoFlexPanels_frontierPosition",
    },
    "scenarioCompareParetoRwPanels": {
        "scenarioCompareParetoRwPanels_pareto",
        "scenarioCompareParetoRwPanels_frontier",
        "scenarioCompareParetoRwPanels_paretoPosition",
        "scenarioCompareParetoRwPanels_frontierPosition",
    },
    "scenarioCompareRwPanels": {
        "scenarioCompareRwPanels_rate_x",
        "scenarioCompareRwPanels_rate_xDifference",
        "scenarioCompareRwPanels_rate_y",
        "scenarioCompareRwPanels_rate_yDifference",
        "scenarioCompareRwPanels_rate_z",
        "scenarioCompareRwPanels_rate_zDifference",
        "scenarioCompareRwPanels_panelPositive",
        "scenarioCompareRwPanels_panelPositive_difference",
        "scenarioCompareRwPanels_panelNegative",
        "scenarioCompareRwPanels_panelNegative_difference",
        "scenarioCompareRwPanels_attError",
    },
    "scenarioCompareTorque": {
        "scenarioCompareTorque_rate_x",
        "scenarioCompareTorque_rate_xDifference",
        "scenarioCompareTorque_rate_y",
        "scenarioCompareTorque_rate_yDifference",
        "scenarioCompareTorque_rate_z",
        "scenarioCompareTorque_rate_zDifference",
        "scenarioCompareTorque_motionAttError",
        "scenarioCompareTorque_attError",
        "scenarioCompareTorque_analyticRateError",
        "scenarioCompareTorque_analyticAttError",
    },
    "scenarioCompareVariableMass": {
        "scenarioCompareVariableMass_orbit",
        "scenarioCompareVariableMass_orbit_difference",
        "scenarioCompareVariableMass_fuelMass",
        "scenarioCompareVariableMass_fuelMass_difference",
        "scenarioCompareVariableMass_rate_x",
        "scenarioCompareVariableMass_rate_xDifference",
        "scenarioCompareVariableMass_rate_y",
        "scenarioCompareVariableMass_rate_yDifference",
        "scenarioCompareVariableMass_rate_z",
        "scenarioCompareVariableMass_rate_zDifference",
        "scenarioCompareVariableMass_slosh_x",
        "scenarioCompareVariableMass_slosh_xDifference",
        "scenarioCompareVariableMass_slosh_y",
        "scenarioCompareVariableMass_slosh_yDifference",
        "scenarioCompareVariableMass_slosh_z",
        "scenarioCompareVariableMass_slosh_zDifference",
        "scenarioCompareVariableMass_attError",
        "scenarioCompareVariableMass_comError",
    },
}

PAPER_SWEEP_RUN_KWARGS = {
    "sweepOrbitDt": {
        "dts": (20.0, 10.0, 5.0),
        "simDuration": 600.0,
        "recordDt": 20.0,
    },
    "sweepTorqueArtifact": {
        "speeds": (0.0, 7500.0),
        "dts": (0.2, 0.1),
        "tf": 0.2,
        "recordDt": 0.1,
        "velocitySweepDt": 0.1,
    },
}

RESEARCH_ENTRY_POINTS = (
    "paperFigures",
    "runAllComparisons",
    "sweepOrbitDt",
    "sweepRwBookkeepingMass",
    "sweepSloshDisplacement",
    "sweepTorqueArtifact",
    "sweepTorqueMechanismChecks",
    "sweepVariableMassPendulumInertia",
)

sys.path.append(SCENARIO_FOLDER)

requiresMujoco = pytest.mark.skipif(
    not hasBuildFeature("mujoco"),
    reason="Basilisk was built without MuJoCo",
)


@pytest.mark.parametrize("moduleName", RESEARCH_ENTRY_POINTS)
def test_research_entry_points_import(moduleName):
    """Import research drivers used by documentation and command-line workflows."""
    importlib.import_module(moduleName)


def test_history_validation_rejects_mismatch_and_short_run():
    """Reject unequal timestamps and histories that stop before the requested horizon."""
    validation = importlib.import_module("_comparisonValidation")
    with pytest.raises(ValueError, match="timestamp histories differ"):
        validation.validateMatchingHistories(
            "test", [0.0, 0.4, 1.0], [0.0, 0.5, 1.0],
            expectedFinalTime=1.0)
    with pytest.raises(ValueError, match="ended at"):
        validation.validateHistory(
            "test", [0.0, 0.5], expectedFinalTime=1.0, sampleInterval=0.5)
    with pytest.raises(ValueError, match="samples"):
        validation.validateHistory(
            "test", [0.0, 1.0], values=np.zeros((1, 3)))
    with pytest.raises(ValueError, match="non-finite values"):
        validation.validateHistory(
            "test", [0.0, 1.0], values=np.array([[0.0], [np.nan]]))


@pytest.mark.parametrize(
    "times,message",
    [
        ([], "nonempty one-dimensional"),
        ([[0.0, 1.0]], "nonempty one-dimensional"),
        ([0.0, np.nan], "non-finite timestamps"),
        ([0.0, 0.5, 0.5], "strictly increasing"),
        ([0.0, 0.6, 0.5], "strictly increasing"),
    ],
)
def test_history_validation_rejects_invalid_timestamps(times, message):
    """Reject malformed timestamps before comparison metrics are computed."""
    validation = importlib.import_module("_comparisonValidation")
    with pytest.raises(ValueError, match=message):
        validation.validateHistory("test", times)


def test_history_validation_accepts_complete_schedule_and_equal_lengths():
    """Return a complete recorder schedule and constrain equal history lengths."""
    validation = importlib.import_module("_comparisonValidation")
    assert validation.recorderSampleInterval(0.2, 0.1) == pytest.approx(0.2)
    assert validation.recorderSampleInterval(40.0, 60.0) == pytest.approx(80.0)
    assert validation.alignedHorizon(
        11.7, (0.4, 0.6)) == pytest.approx(10.8)  # [s]
    times = validation.validateHistory(
        "test",
        [0.0, 0.5, 1.0],
        expectedFinalTime=1.0,
        sampleInterval=0.5,
        values=np.zeros((3, 2)),
    )
    np.testing.assert_array_equal(times, [0.0, 0.5, 1.0])
    assert validation.requireEqualLength("test", [1, 2], np.zeros((2, 3))) == 2
    with pytest.raises(ValueError, match="at least one history"):
        validation.requireEqualLength("test")
    with pytest.raises(ValueError, match="history lengths differ"):
        validation.requireEqualLength("test", [1], [1, 2])


def test_orbit_sweep_rejects_incomplete_engine_history():
    """Reject a sweep rung whose engines reach the horizon on different schedules."""
    module = importlib.import_module("sweepOrbitDt")
    bsmTimes = np.array([0.0, 1.0, 2.0])  # [s]
    mujocoTimes = np.array([0.0, 2.0])  # [s]
    with pytest.raises(ValueError, match="samples"):
        module.comparisonRow(
            0.1, 2.0, 1.0,
            bsmTimes, mujocoTimes,
            np.zeros((3, 3)), np.zeros((2, 3)), np.zeros((3, 3)),
        )

    with pytest.raises(ValueError, match="ended at"):
        module.comparisonRow(
            0.1, 2.0, 1.0,
            bsmTimes[:-1], bsmTimes[:-1],
            np.zeros((2, 3)), np.zeros((2, 3)), np.zeros((2, 3)),
        )


@requiresMujoco
@pytest.mark.parametrize(
    "moduleName,runKwargs",
    [
        pytest.param(
            "sweepOrbitDt",
            {"dts": (20.0, 10.0, 5.0), "simDuration": 600.0, "recordDt": 20.0},
            id="orbit-step",
        ),
        pytest.param(
            "sweepRwBookkeepingMass",
            {
                "rwMasses": (1.0e-4, 1.0e-6, 1.0e-8),  # [kg]
                "dt": 0.02,
                "tf": 2.0,
                "recordDt": 0.1,
            },
            id="reaction-wheel-mass",
        ),
        pytest.param(
            "sweepSloshDisplacement",
            {"configurations": (False, True), "tf": 0.1},
            id="slosh-displacement",
        ),
        pytest.param(
            "sweepTorqueArtifact",
            {
                "speeds": (0.0, 7500.0),
                "dts": (0.2, 0.1),
                "tf": 0.2,
                "recordDt": 0.1,
                "velocitySweepDt": 0.1,
            },
            id="torque-artifact",
        ),
        pytest.param(
            "sweepTorqueMechanismChecks",
            {
                "speeds": (3750.0, 7500.0),
                "masses": (750.0, 1500.0),
                "timeStep": 0.1,
                "simDuration": 0.1,
                "recordStep": 0.1,
            },
            id="torque-mechanism",
        ),
        pytest.param(
            "sweepVariableMassPendulumInertia",
            {
                "bobInertias": (1.0e-8, 1.0e-6, 1.0e-4),
                "timeSteps": (0.05,),
                "simDuration": 1.0,
            },
            id="variable-mass-inertia",
        ),
    ],
)
def test_research_sweeps_capture_expected_physical_trends(
        moduleName, runKwargs):
    """Execute reduced real sweeps and constrain their physical trends."""
    module = importlib.import_module(moduleName)
    result = module.run(saveJson=False, **runKwargs)

    if moduleName == "sweepOrbitDt":
        metrics = result
        assert np.allclose(
            metrics["pairwiseTruncationOrders"], 4.0, rtol=0.03)
    elif moduleName == "sweepRwBookkeepingMass":
        metrics = result
        masses = np.array([row["rwMass"] for row in metrics["rows"]])
        floors = np.array([
            row["hubAttitudePrincipalAngleMax"] for row in metrics["rows"]
        ])
        assert np.all(floors > 0.0)
        assert np.all(np.diff(floors) < 0.0)
        # Compare decades because the smallest residual is near round-off.
        floorDecadeChanges = np.diff(np.log10(floors))
        massDecadeChanges = np.diff(np.log10(masses))
        np.testing.assert_allclose(
            floorDecadeChanges,
            massDecadeChanges,
            atol=0.02,
            rtol=0.0,
        )
    elif moduleName == "sweepSloshDisplacement":
        slosh = result["maxAbsRhoDiff_m"]
        assert np.all(np.isfinite(list(slosh.values())))
        assert slosh["inOrbit"] > slosh["deepSpace"]
    elif moduleName == "sweepTorqueArtifact":
        metrics = result
        assert (
            metrics["velocitySweep"][-1]["mujocoDriftAttMax"]
            > 1.0e6*metrics["velocitySweep"][0]["mujocoDriftAttMax"]
        )
        assert (
            metrics["dtSweep"][-1]["crossEngineRestAttMax"]
            < 0.1*metrics["dtSweep"][0]["crossEngineRestAttMax"]
        )
    elif moduleName == "sweepTorqueMechanismChecks":
        metrics = result
        assert np.all(np.isfinite(metrics["genericInitialBiasForces"]))
        genericBias = np.linalg.norm(metrics["genericInitialBiasForces"])
        axisBias = np.linalg.norm(metrics["axisInitialBiasForces"])
        assert genericBias > 0.0
        assert axisBias < 1.0e-8*genericBias
        assert metrics["quietAxisAttitudeMax"] < 1.0e-8*(
            metrics["quietVelocityScaling"][-1]["mujocoAttitudeMax"])
        assert metrics["quietVelocityScaling"][-1]["ratioToPrevious"] == pytest.approx(
            4.0, rel=1.0e-6)
        assert metrics["quietMassScaling"][-1]["ratioToPrevious"] == pytest.approx(
            2.0, rel=1.0e-6)
    else:
        axis = result[module.fileName].axes[0]
        errors = np.asarray(axis.lines[0].get_ydata())
        assert np.all(np.isfinite(errors))
        assert np.all(np.diff(errors) > 0.0)
        module.plt.close("all")


def _assert_variable_mass_orbit_metrics(metrics):
    """Bound the published orbit case while preserving its gravity-model difference."""
    assert 0.11 < metrics["propellantDepletedFraction"] < 0.12
    assert metrics["initialThrustTorqueMagnitude"] == pytest.approx(
        0.1049, rel=0.01)  # [N*m]
    assert 75.0 < metrics["deltaV"] < 85.0  # [m/s]
    assert 2.3e5 < metrics["semiMajorAxisRise"] < 2.6e5  # [m]
    assert 3.0e-2 < metrics["attitudeErrorMax"] < 5.0e-2  # [rad]
    assert 200.0 < metrics["comPositionErrorMax"] < 300.0  # [m]
    assert 1.0e-4 < metrics["rateErrorMax"] < 1.5e-4  # [rad/s]
    assert metrics["totalMassErrorMax"] < 2.0e-3  # [kg]
    assert metrics["sloshDisplacementErrorMax"] < 2.0e-5  # [m]
    assert metrics["pendulumAngleErrorMax"] < 3.0e-5  # [rad]
    assert 2.3e5 < metrics["semiMajorAxisRiseMujoco"] < 2.6e5  # [m]
    assert 700.0 < metrics["semiMajorAxisDifferenceFinal"] < 950.0  # [m]
    assert (
        2.5e-3
        < metrics["semiMajorAxisDifferenceAsFractionOfRise"]
        < 4.5e-3
    )


def _assert_orbit_metrics(metrics):
    """Constrain the analytic orbit accuracy and cross-engine residual."""
    assert metrics["bsmVsAnalyticMax"] == pytest.approx(
        0.0304681, rel=0.02
    )  # [m]
    assert metrics["mujocoVsAnalyticMax"] == pytest.approx(
        0.0304680, rel=0.02
    )  # [m]
    assert metrics["crossParadigmPosMax"] < 2.0e-7  # [m]
    assert metrics["bsmEnergyDrift"] < 6.0e-11
    assert metrics["mujocoEnergyDrift"] < 6.0e-11


def _assert_torque_metrics(metrics):
    """Constrain the analytic torque response and the orbital motion artifact."""
    assert metrics["bsmAnalyticRateErrorMax"] < 5.0e-13  # [rad/s]
    assert metrics["mujocoAnalyticRateErrorMax"] < 5.0e-13  # [rad/s]
    assert metrics["bsmAnalyticAttitudeErrorMax"] < 1.0e-8  # [rad]
    assert metrics["mujocoAnalyticAttitudeErrorMax"] < 1.0e-7  # [rad]
    assert metrics["bsmMotionAttitudeMax"] < 1.0e-14  # [rad]
    assert metrics["crossEngineAttitudeRestMax"] < 1.0e-6  # [rad]
    assert metrics["crossEngineAttitudeOrbitMax"] < 1.0e-4  # [rad]
    assert (
        metrics["crossEngineAttitudeOrbitMax"]
        > 10.0*metrics["crossEngineAttitudeRestMax"]
    )
    assert metrics["bodyRateErrorMax"] < 1.0e-6  # [rad/s]


def _assert_rw_panel_metrics(metrics):
    """Constrain the coupled wheel, hub, and panel response."""
    assert metrics["hubAttitudePrincipalAngleMax"] < 2.0e-11  # [rad]
    assert metrics["hubBodyRateErrorMax"] < 1.0e-12  # [rad/s]
    assert metrics["wheelSpeedErrorMax"] < 1.0e-12  # [rad/s]
    assert metrics["panelAngleErrorMax"] < 1.0e-9  # [rad]


def _assert_flex_panel_metrics(metrics):
    """Constrain the reduced articulated-chain sweep used in CI."""
    rows = metrics["rows"]
    for row in rows:
        assert row["ndofVsNHingedMax"] < 1.0e-12  # [rad]
        assert row["attitudeErrorMax"] < 1.0e-12  # [rad]
        assert row["ndofAttitudeErrorMax"] < 1.0e-12  # [rad]
        for key in ("bsmWall", "ndofWall", "mujocoWall"):
            assert np.isfinite(row[key])
            assert row[key] > 0.0  # [s]


def _assert_orbit_multibody_metrics(metrics):
    """Constrain compact agreement and the extended differential-gravity case."""
    compact = metrics["compact"]
    extended = metrics["extended"]
    assert compact["bsmVsKeplerMax"] < 2.0e-7  # [m]
    assert extended["bsmVsKeplerMax"] < 2.0e-6  # [m]
    assert compact["crossParadigmOrbitMax"] < 2.0e-7  # [m]
    assert compact["crossParadigmAttitudeMax"] < 2.0e-4  # [rad]
    assert extended["crossParadigmOrbitMax"] < 2.0e-6  # [m]
    assert 0.7 < extended["crossParadigmAttitudeMax"] < 0.9  # [rad]
    assert (
        extended["crossParadigmAttitudeMax"]
        > 1.0e3*compact["crossParadigmAttitudeMax"]
    )


def _assert_pareto_metrics(metrics):
    """Validate reduced work-precision rows without constraining wall-clock noise."""
    for engine in ("bsm", "mujoco"):
        rows = metrics[engine]
        for row in rows:
            assert np.isfinite(row["error"])
            assert np.isfinite(row["positionError"])
            assert np.isfinite(row["wall"])
            assert 0.0 <= row["error"] < 0.1  # [rad]
            assert 0.0 <= row["positionError"] < 1.0  # [m]
            assert row["wall"] > 0.0  # [s]
        reference = metrics["referenceSelfConsistency"][engine]
        assert np.isfinite(reference["attitude"])
        assert np.isfinite(reference["position"])
        assert 0.0 <= reference["attitude"] < 1.0e-8  # [rad]
        assert 0.0 <= reference["position"] < 1.0e-7  # [m]
        errors = np.array([row["error"] for row in rows])  # [rad]
        positionErrors = np.array([
            row["positionError"] for row in rows
        ])  # [m]
        assert np.max(errors) > 2.0*np.min(errors)
        assert np.max(positionErrors) > 2.0*np.min(positionErrors)

        if all(row["tol"] is not None for row in rows):
            byTolerance = {float(row["tol"]): row for row in rows}
            assert byTolerance[1.0e-6]["error"] < byTolerance[1.0e-4]["error"]
            assert (
                byTolerance[1.0e-6]["positionError"]
                < byTolerance[1.0e-4]["positionError"]
            )
        else:
            byConfig = {
                (row["integrator"], float(row["dt"])): row
                for row in rows
            }
            rk4 = byConfig[("svIntegratorRK4", 0.1)]
            euler = byConfig[("svIntegratorEuler", 0.02)]
            assert euler["error"] > rk4["error"]
            assert euler["positionError"] > rk4["positionError"]


SCENARIO_METRIC_ASSERTIONS = {
    "scenarioCompareFlexPanels": _assert_flex_panel_metrics,
    "scenarioCompareOrbit": _assert_orbit_metrics,
    "scenarioCompareOrbitMultibody": _assert_orbit_multibody_metrics,
    "scenarioCompareParetoFlexPanels": _assert_pareto_metrics,
    "scenarioCompareParetoRwPanels": _assert_pareto_metrics,
    "scenarioCompareRwPanels": _assert_rw_panel_metrics,
    "scenarioCompareTorque": _assert_torque_metrics,
    "scenarioCompareVariableMass": _assert_variable_mass_orbit_metrics,
}


def _assert_figure_data(figureList, expectedNames):
    """Require every documented figure and at least one finite plotted series."""
    assert set(figureList) == expectedNames
    for figureName, figure in figureList.items():
        plottedPoints = 0
        for axes in figure.axes:
            for line in axes.lines:
                xData = np.asarray(line.get_xdata())
                yData = np.asarray(line.get_ydata())
                assert xData.size == yData.size
                assert np.all(np.isfinite(xData))
                assert np.all(np.isfinite(yData))
                plottedPoints += xData.size
            for collection in axes.collections:
                offsets = np.asarray(collection.get_offsets())
                if offsets.size:
                    assert np.all(np.isfinite(offsets))
                    plottedPoints += len(offsets)
        assert plottedPoints > 0, f"{figureName} contains no plotted data"


@pytest.mark.parametrize("scenario", SCENARIO_FILES)
@pytest.mark.scenarioTest
def test_scenarios(scenario: str):
    """Run a real dynamics comparison and constrain its numerical result.

    Args:
        scenario (str): module name of the scenario file to import and run.
    """
    module = importlib.import_module(scenario)
    if hasattr(module, "couldImportMujoco") and not module.couldImportMujoco:
        pytest.skip("Dynamics comparisons require a MuJoCo-enabled Basilisk build")
    kwargs = dict(SCENARIO_RUN_KWARGS.get(scenario, {}))
    with tempfile.TemporaryDirectory(prefix=f"bsk-{scenario}-") as resultsDir:
        figureList = module.run(
            showPlots=False,
            saveJson=True,
            resultsDir=resultsDir,
            **kwargs,
        )
        resultPath = os.path.join(resultsDir, scenario+".json")
        with open(resultPath) as stream:
            metrics = json.load(stream)
        SCENARIO_METRIC_ASSERTIONS[scenario](metrics)
        _assert_figure_data(figureList, SCENARIO_EXPECTED_FIGURES[scenario])


@requiresMujoco
def test_aggregate_runner_executes_real_scenario():
    """Run one scenario through the aggregate entry point and persist its artifacts."""
    runner = importlib.import_module("runAllComparisons")
    with tempfile.TemporaryDirectory(prefix="bsk-runner-") as resultsDir:
        runner.run(
            scenarios=("scenarioCompareOrbit",),
            scenarioRunKwargs={
                "scenarioCompareOrbit": {"saveTiming": False}
            },
            resultsDir=resultsDir,
            saveDocumentationFigures=False,
        )

        resultPath = os.path.join(resultsDir, "scenarioCompareOrbit.json")
        figurePath = os.path.join(
            resultsDir, "scenarioCompareOrbit_trajectory.svg")
        with open(resultPath) as stream:
            _assert_orbit_metrics(json.load(stream))
        assert os.path.getsize(figurePath) > 1000


@requiresMujoco
def test_paper_pipeline_renders_and_revalidates_real_data():
    """Regenerate reduced real inputs and execute the deterministic paper pipeline."""
    module = importlib.import_module("paperFigures")
    scenarioOverrides = {
        name: SCENARIO_RUN_KWARGS[name]
        for name in module.PAPER_SCENARIOS
    }
    with tempfile.TemporaryDirectory(prefix="bsk-paper-") as workDir:
        resultsDir = os.path.join(workDir, "results")
        outputDir = os.path.join(workDir, "paper")
        module.regeneratePublicationInputs(
            scenarioRunKwargs=scenarioOverrides,
            sweepRunKwargs=PAPER_SWEEP_RUN_KWARGS,
            resultsDir=resultsDir,
        )

        module._renderFiguresInFreshProcess(outputDir, resultsDir)
        # Exercise rendering and hashes here; publication mode enforces the
        # clean-build gate separately.
        head = module._gitOutput("rev-parse", "HEAD")
        buildInfo = module._getBuildInfo()
        buildReceipt = module._currentBuildReceipt(
            head, buildInfo, cleanRebuild=False)
        provenance = module._publicationProvenance(
            head, buildInfo, outputDir, buildReceipt, resultsDir,
            dataFiles=module.PAPER_INPUT_FILES)
        provenanceFile = os.path.join(outputDir, "provenance.json")
        module._writeProvenance(provenance, provenanceFile)

        for name in module.PAPER_OUTPUT_FILES:
            os.remove(os.path.join(outputDir, name))
        with module.matplotlib.rc_context({"pdf.compression": 0}):
            module.run(
                outputDir=outputDir,
                provenanceFile=provenanceFile,
                verifySource=False,
                resultsDir=resultsDir,
            )
        module.validateFigureProvenance(provenance, outputDir)

        inputPath = os.path.join(resultsDir, module.PAPER_INPUT_FILES[0])
        with open(inputPath, "rb") as stream:
            originalInput = stream.read()
        try:
            with open(inputPath, "ab") as stream:
                stream.write(b"\n")
            with pytest.raises(ValueError, match="hash mismatch"):
                module.run(
                    outputDir=outputDir,
                    provenanceFile=provenanceFile,
                    verifySource=False,
                    resultsDir=resultsDir,
                )
        finally:
            with open(inputPath, "wb") as stream:
                stream.write(originalInput)


def test_variable_mass_rejects_nondivisible_horizon():
    """Reject a requested final epoch that cannot lie on the integration grid."""
    module = importlib.import_module("scenarioCompareVariableMass")
    with pytest.raises(ValueError, match="not divisible"):
        module.run(
            showPlots=False,
            saveJson=False,
            simDuration=1.03,
            useThruster=False,
            inOrbit=False,
        )


@requiresMujoco
def test_variable_mass_leak_depletes_without_external_impulse():
    """Deplete propellant without introducing thrust force or moment."""
    module = importlib.import_module("scenarioCompareVariableMass")

    with tempfile.TemporaryDirectory(prefix="bsk-variable-leak-") as resultsDir:
        module.run(
            showPlots=False,
            saveJson=True,
            simDuration=1.0,
            useThruster=False,
            inOrbit=False,
            resultsDir=resultsDir,
        )

        resultPath = os.path.join(
            resultsDir, "scenarioCompareVariableMass_deepSpace.json")
        with open(resultPath) as stream:
            metrics = json.load(stream)
        assert metrics["propellantDepletedFraction"] > 1.0e-4
        assert abs(metrics["deltaV"]) < 1.0e-5  # [m/s]
        assert metrics["initialThrustTorqueMagnitude"] == 0.0
    module.plt.close("all")


def test_variable_mass_thruster_burn_covers_requested_horizon():
    """Continue depleting propellant beyond the former 1800-second command limit."""
    module = importlib.import_module("scenarioCompareVariableMass")
    simDuration = 1801.0  # [s]
    timeStep = module.timeStep()  # [s]
    simulation, recorders, _ = module.buildBSM(
        timeStep,
        record=True,
        useThruster=True,
        inOrbit=False,
        simDuration=simDuration,
    )
    simulation.ConfigureStopTime(macros.sec2nano(simDuration))
    simulation.ExecuteSimulation()
    histories = module.pullBSM(recorders, module.earthMu())

    time = histories["t"]  # [s]
    speed = np.linalg.norm(histories["v_CN_N"], axis=1)  # [m/s]
    limitIndex = int(np.searchsorted(time, 1800.0))
    precedingRise = speed[limitIndex] - speed[limitIndex-20]  # [m/s]
    followingRise = speed[-1] - speed[limitIndex]  # [m/s]
    assert followingRise > 0.8*precedingRise


@requiresMujoco
def test_articulated_scenarios_have_nontrivial_absolute_motion():
    """Anchor coupled-model agreement to independent BSM response magnitudes."""
    rwPanels = importlib.import_module("scenarioCompareRwPanels")
    bsmState, bsmWheel, bsmPanels, _ = rwPanels.runBSM(
        0.02, 120.0, 0.5
    )  # [s]
    bodyRate = np.asarray(bsmState.omega_BN_B)  # [rad/s]
    wheelSpeed = np.asarray(bsmWheel.wheelSpeeds)[:, :3]  # [rad/s]
    panelAngle = np.column_stack([
        np.squeeze(np.asarray(recorder.theta))
        for recorder in bsmPanels
    ])  # [rad]
    assert np.max(np.linalg.norm(bodyRate, axis=1)) == pytest.approx(
        0.0269258, rel=0.01
    )  # [rad/s]
    assert np.max(np.abs(wheelSpeed-wheelSpeed[0])) == pytest.approx(
        22.5066, rel=0.01
    )  # [rad/s]
    assert np.max(np.abs(panelAngle-panelAngle[0])) == pytest.approx(
        0.391224, rel=0.01
    )  # [rad]

    flexPanels = importlib.import_module("scenarioCompareFlexPanels")
    timeStep = flexPanels.timeStep(1)  # [s]
    simulation, stateRecorder, _ = flexPanels.buildBSM(1, timeStep, True)
    simulation.ConfigureStopTime(macros.sec2nano(8.0))  # [s]
    simulation.ExecuteSimulation()
    attitude = np.asarray(stateRecorder.sigma_BN)
    initialAttitude = np.repeat(attitude[:1], len(attitude), axis=0)
    excursion = np.max(
        flexPanels.relativePrincipalAngle(attitude, initialAttitude)
    )  # [rad]
    assert excursion == pytest.approx(0.215009, rel=0.01)


def test_pareto_flex_requires_common_comparison_horizon():
    """Reject steps that would compare states from different physical times."""
    module = importlib.import_module("scenarioCompareParetoFlexPanels")
    with pytest.raises(ValueError, match="does not divide"):
        module.finalState(
            module.buildBSM, "svIntegratorRK4", 3.0e-4, None,
            simDuration=1.0e-3)
    stopTimeNanos = macros.sec2nano(module.SIM_DURATION)
    for _, dt, _ in module.FULL_SWEEP_CONFIGS:
        assert stopTimeNanos % macros.sec2nano(dt) == 0


def test_pareto_rw_requires_common_comparison_horizon():
    """Reject reaction-wheel Pareto steps that stop before the common epoch."""
    module = importlib.import_module("scenarioCompareParetoRwPanels")
    with pytest.raises(ValueError, match="does not divide"):
        module.finalState(module.buildBSM, "svIntegratorRK4", 0.8, None)
    assert module.snappedStep(0.8) == pytest.approx(0.75)  # [s]
    execution, records = module.normalizedSweepConfigurations(
        (("svIntegratorRK4", 0.8, None),))
    assert execution == (("svIntegratorRK4", 0.75, None),)
    assert records == [{
        "integrator": "svIntegratorRK4",
        "dt": 0.75,
        "tol": None,
        "requestedDt": 0.8,
    }]
    stopTimeNanos = macros.sec2nano(module.SIM_DURATION)
    for _, dt, _ in module.FULL_SWEEP_CONFIGS:
        assert stopTimeNanos % macros.sec2nano(dt) == 0


@pytest.mark.parametrize("axis", (0, 1, 2))
@requiresMujoco
def test_variable_mass_native_pendulum_damping_conserves_momentum(axis):
    r"""Validate transverse decay, undamped rod spin, and internal action-reaction torque.

    A free parent and concentric ball-jointed child are isotropic and initially rotated about the
    tested child axis. Native zero-command velocity servos damp x and y, while z represents the
    pendulum rod-spin axis and remains undamped. In all cases inertial angular momentum is
    conserved.
    """
    module = importlib.import_module("scenarioCompareVariableMass")
    timeStep = 0.001  # [s]
    finalTime = 0.5  # [s]
    parentInertia = 2.0  # [kg*m^2]
    childInertia = 1.0  # [kg*m^2]
    damping = 0.2  # [N*m*s]
    initialRate = 0.4  # [rad/s]
    quaternions = (
        "0.9659258263 0.2588190451 0 0",
        "0.9659258263 0 0.2588190451 0",
        "0.9659258263 0 0 0.2588190451",
    )
    childQuaternions = (
        "0.9848077530 0.1736481777 0 0",
        "0.9848077530 0 0.1736481777 0",
        "0.9848077530 0 0 0.1736481777",
    )

    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    process.addTask(simulation.CreateNewTask(
        "testTask", macros.sec2nano(timeStep)))
    scene = module.mujoco.MJScene(
        f"""
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="parent" quat="{quaternions[axis]}">
      <freejoint/>
      <inertial pos="0 0 0" mass="1"
                diaginertia="{parentInertia} {parentInertia} {parentInertia}"/>
      <body name="child" quat="{childQuaternions[axis]}">
        <joint name="ball" type="ball"/>
        <inertial pos="0 0 0" mass="1"
                  diaginertia="{childInertia} {childInertia} {childInertia}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity name="dampX" joint="ball" gear="1 0 0" kv="{damping}"/>
    <velocity name="dampY" joint="ball" gear="0 1 0" kv="{damping}"/>
  </actuator>
</mujoco>
"""
    )
    simulation.AddModelToTask("testTask", scene)
    parentRecorder = scene.getBody("parent").getOrigin().stateOutMsg.recorder()
    childRecorder = scene.getBody("child").getOrigin().stateOutMsg.recorder()
    simulation.AddModelToTask("testTask", parentRecorder)
    simulation.AddModelToTask("testTask", childRecorder)
    simulation.InitializeSimulation()

    ball = scene.getBody("child").getBallJoint()
    qvelState = scene.dynManager.getStateObject("mujocoQvel")
    qvel = np.asarray(qvelState.getState()).reshape(-1)
    qvel[ball.getQvelAdr() + axis] = initialRate  # [rad/s]
    qvelState.setState(qvel.reshape(-1, 1))

    simulation.ConfigureStopTime(macros.sec2nano(finalTime))
    simulation.ExecuteSimulation()

    parentRate = np.asarray(parentRecorder.omega_BN_B)  # [rad/s]
    childRate = np.asarray(childRecorder.omega_BN_B)  # [rad/s]
    parentSigma = np.asarray(parentRecorder.sigma_BN)
    childSigma = np.asarray(childRecorder.sigma_BN)
    parentRateN = np.array([
        rbk.MRP2C(parentSigma[i]).T @ parentRate[i]
        for i in range(len(parentRate))
    ])  # [rad/s]
    childRateN = np.array([
        rbk.MRP2C(childSigma[i]).T @ childRate[i]
        for i in range(len(childRate))
    ])  # [rad/s]

    momentumN = (
        parentInertia*parentRateN + childInertia*childRateN
    )  # [N*m*s]
    np.testing.assert_allclose(
        momentumN,
        np.broadcast_to(momentumN[0], momentumN.shape),
        rtol=0.0,
        atol=2.0e-10,  # [N*m*s]
    )

    axisN = rbk.MRP2C(childSigma[0]).T @ np.eye(3)[axis]
    finalRelativeRate = (childRateN[-1] - parentRateN[-1]) @ axisN  # [rad/s]
    if axis < 2:
        decayRate = damping*(
            1.0/parentInertia + 1.0/childInertia)  # [1/s]
        expectedRate = initialRate*np.exp(-decayRate*finalTime)  # [rad/s]
        assert finalRelativeRate == pytest.approx(expectedRate, rel=2.0e-3)
    else:
        assert finalRelativeRate == pytest.approx(initialRate, abs=2.0e-12)


@requiresMujoco
def test_variable_mass_native_damping_in_rotated_anisotropic_system():
    r"""Check the transverse damping power in noncoaxial child and parent frames.

    The bodies are concentric but anisotropic, with unrelated initial orientations and an oblique
    ball-joint rate. The native x/y velocity servos must dissipate
    :math:`k_v(\omega_x^2+\omega_y^2)` at the initial instant while preserving inertial angular
    momentum. This case exposes child/parent-frame confusion hidden by the axis-aligned tests.
    """
    module = importlib.import_module("scenarioCompareVariableMass")
    timeStep = 1.0e-4  # [s]
    finalTime = 2.0e-2  # [s]
    damping = 0.2  # [N*m*s]
    parentInertia = np.array([2.0, 2.5, 3.0])  # [kg*m^2]
    childInertia = np.array([1.0, 1.2, 1.4])  # [kg*m^2]
    initialRelativeRate = np.array([0.4, -0.3, 0.2])  # [rad/s]

    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("testProcess")
    process.addTask(simulation.CreateNewTask(
        "testTask", macros.sec2nano(timeStep)))
    scene = module.mujoco.MJScene(
        f"""
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="parent" quat="0.9238795325 0.2209423827 0.2209423827 0.2209423827">
      <freejoint/>
      <inertial pos="0 0 0" mass="1"
                diaginertia="{parentInertia[0]} {parentInertia[1]} {parentInertia[2]}"/>
      <body name="child" quat="0.9659258263 -0.1830127019 0.1830127019 0">
        <joint name="ball" type="ball"/>
        <inertial pos="0 0 0" mass="1"
                  diaginertia="{childInertia[0]} {childInertia[1]} {childInertia[2]}"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <velocity name="dampX" joint="ball" gear="1 0 0" kv="{damping}"/>
    <velocity name="dampY" joint="ball" gear="0 1 0" kv="{damping}"/>
  </actuator>
</mujoco>
"""
    )
    simulation.AddModelToTask("testTask", scene)
    parentRecorder = scene.getBody("parent").getOrigin().stateOutMsg.recorder()
    childRecorder = scene.getBody("child").getOrigin().stateOutMsg.recorder()
    simulation.AddModelToTask("testTask", parentRecorder)
    simulation.AddModelToTask("testTask", childRecorder)
    simulation.InitializeSimulation()

    ball = scene.getBody("child").getBallJoint()
    qvelState = scene.dynManager.getStateObject("mujocoQvel")
    qvel = np.asarray(qvelState.getState()).reshape(-1)
    qvel[ball.getQvelAdr():ball.getQvelAdr() + 3] = initialRelativeRate
    qvelState.setState(qvel.reshape(-1, 1))

    simulation.ConfigureStopTime(macros.sec2nano(finalTime))
    simulation.ExecuteSimulation()

    parentRate = np.asarray(parentRecorder.omega_BN_B)  # [rad/s]
    childRate = np.asarray(childRecorder.omega_BN_B)  # [rad/s]
    parentSigma = np.asarray(parentRecorder.sigma_BN)
    childSigma = np.asarray(childRecorder.sigma_BN)
    momentumN = np.array([
        rbk.MRP2C(parentSigma[i]).T @ (parentInertia*parentRate[i])
        + rbk.MRP2C(childSigma[i]).T @ (childInertia*childRate[i])
        for i in range(len(parentRate))
    ])  # [N*m*s]
    np.testing.assert_allclose(
        momentumN,
        np.broadcast_to(momentumN[0], momentumN.shape),
        rtol=0.0,
        atol=2.0e-10,  # [N*m*s]
    )

    energy = np.array([
        0.5*parentRate[i] @ (parentInertia*parentRate[i])
        + 0.5*childRate[i] @ (childInertia*childRate[i])
        for i in range(len(parentRate))
    ])  # [J]
    expectedInitialPower = -damping*np.sum(
        initialRelativeRate[:2]**2)  # [W]
    measuredInitialPower = (energy[1] - energy[0])/timeStep  # [W]
    assert measuredInitialPower == pytest.approx(
        expectedInitialPower, rel=2.0e-3)
    assert energy[-1] < energy[0]


@requiresMujoco
def test_variable_mass_deep_space_numerical_agreement():
    """Constrain the full-slosh cross-engine match without fitted loads."""
    module = importlib.import_module("scenarioCompareVariableMass")

    with tempfile.TemporaryDirectory(prefix="bsk-variable-space-") as resultsDir:
        module.run(
            showPlots=False,
            saveJson=True,
            simDuration=900.0,  # [s]
            useThruster=True,
            inOrbit=False,
            resultsDir=resultsDir,
        )
        resultPath = os.path.join(
            resultsDir, "scenarioCompareVariableMass_deepSpace.json")
        with open(resultPath) as stream:
            full = json.load(stream)
        assert 0.11 < full["propellantDepletedFraction"] < 0.12
        assert 130.0 < full["deltaV"] < 140.0  # [m/s]
        assert full["attitudeErrorMax"] < 5.0e-7  # [rad]
        assert full["rateErrorMax"] < 2.0e-7  # [rad/s]
        assert full["comPositionErrorMax"] < 5.0e-2  # [m]
        assert full["totalMassErrorMax"] < 2.0e-3  # [kg]
        assert full["sloshDisplacementErrorMax"] < 1.0e-3  # [m]
        assert full["pendulumAngleErrorMax"] < 1.0e-3  # [rad]

        module.run(
            showPlots=False,
            saveJson=True,
            simDuration=120.0,  # [s]
            useThruster=True,
            inOrbit=False,
            nearRigid=True,
            resultsDir=resultsDir,
        )
        rigidPath = os.path.join(
            resultsDir, "scenarioCompareVariableMass_deepSpace_rigid.json")
        with open(rigidPath) as stream:
            rigid = json.load(stream)
        assert rigid["nearRigid"] is True
        assert 0.015 < rigid["propellantDepletedFraction"] < 0.016
        assert 0.0075 < rigid["wetMassChangeFraction"] < 0.008
        assert 17.0 < rigid["deltaV"] < 19.0  # [m/s]
        assert rigid["initialPendulumMass"] == pytest.approx(1.0e-6)  # [kg]
        assert rigid["initialSmdMass"] == pytest.approx(1.0e-9)  # [kg]
        assert rigid["attitudeErrorMax"] < 1.0e-10  # [rad]
        assert rigid["rateErrorMax"] < 1.0e-11  # [rad/s]
        assert rigid["comPositionErrorMax"] < 1.0e-5  # [m]
        assert rigid["totalMassErrorMax"] < 1.0e-8  # [kg]


@pytest.mark.skipif(
    not importlib.import_module("scenarioCompareOrbitMultibody").couldImportMujoco,
    reason="Basilisk was built without MuJoCo",
)
def test_orbit_multibody_compares_total_center_of_mass():
    """The two engines must start from the same total-CoM position and velocity."""
    module = importlib.import_module("scenarioCompareOrbitMultibody")
    mu = module.simIncludeGravBody.BODY_DATA["earth"].mu
    dt = 0.01
    bsmState, _, _ = module.runBSM(True, mu, dt, dt, dt)
    _, mjCoM, _, _ = module.runMujoco(True, mu, dt, dt, dt)

    assert np.linalg.norm(
        np.asarray(bsmState.r_CN_N[0]) - np.asarray(mjCoM.r_CN_N[0])
    ) < 1.0e-6
    assert np.linalg.norm(
        np.asarray(bsmState.v_CN_N[0]) - np.asarray(mjCoM.v_CN_N[0])
    ) < 1.0e-8


if __name__ == "__main__":
    pytest.main([__file__])
