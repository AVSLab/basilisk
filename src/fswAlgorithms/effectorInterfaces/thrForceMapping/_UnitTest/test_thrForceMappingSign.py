# ISC License
#
# Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
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

"""Validate off-pulsing sign repair independently of the legacy reference model."""

import numpy as np
import pytest
from scipy.optimize import minimize, nnls

from Basilisk.architecture import messaging
from Basilisk.fswAlgorithms import thrForceMapping
from Basilisk.utilities import SimulationBaseClass, fswSetupThrusters, macros


def run_mapping(mapping, target, axes, limits, threshold=4.0, request=None):  # [rad]
    """Run an off-pulsing allocation for a prescribed body-frame torque map.

    :param mapping: torque map with one column per thruster [m]
    :param target: nominal-thrust-compensated body torque [N*m]
    :param axes: orthonormal control axes, one per row
    :param limits: maximum thrust per thruster [N]
    :param threshold: saturation scaling angle threshold [rad]
    :param request: optional exact input torque, avoiding compensation roundoff in boundary tests [N*m]
    :return: signed forces [N] and reported torque angle [rad]
    """
    sim = SimulationBaseClass.SimBaseClass()
    sim.CreateNewProcess("process").addTask(sim.CreateNewTask("task", macros.sec2nano(1.0)))  # [s]
    module = thrForceMapping.thrForceMapping()
    module.ModelTag = "offPulsingSignTest"
    module.thrForceSign = -1
    module.epsilon = 1e-12  # [N*m] zero-torque threshold; also used by the legacy determinant test
    module.angErrThresh = threshold
    module.controlAxes_B = axes.ravel().tolist() + [0.0] * (9 - axes.size)

    # Construct physical positions/directions whose cross products give the requested columns.
    fswSetupThrusters.clearSetup()
    for column, limit in zip(mapping.T, limits):
        if column[2] == 0.0:
            direction = np.array([0.0, 0.0, 1.0])
        else:
            reference = np.eye(3)[np.argmin(np.abs(column))]
            direction = np.cross(column, reference)
            direction /= np.linalg.norm(direction)
        position = np.cross(direction, column)  # [m]
        fswSetupThrusters.create(position, direction, limit)
    thrusters = fswSetupThrusters.writeConfigMessage()
    vehicle = messaging.VehicleConfigMsg().write(messaging.VehicleConfigMsgPayload())
    torque_request = target + mapping @ limits if request is None else np.asarray(request)
    torque = messaging.CmdTorqueBodyMsg().write(
        messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=torque_request.tolist())
    )
    module.thrConfigInMsg.subscribeTo(thrusters)
    module.vehConfigInMsg.subscribeTo(vehicle)
    module.cmdTorqueInMsg.subscribeTo(torque)
    recorder = module.thrForceCmdOutMsg.recorder()
    sim.AddModelToTask("task", module)
    sim.AddModelToTask("task", recorder)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(0)  # [ns]
    sim.ExecuteSimulation()
    return np.array(recorder.thrForce[-1, :len(limits)]), module.outTorqAngErr


def dv_case(request):
    """Return the existing six-thruster DV geometry with the reported CoM offset.

    :param request: input body torque [N*m]
    :return: body torque map [m], compensated target [N*m], and thrust limits [N]
    """
    positions = np.array([
        [0.0, 0.413, -0.1671], [0.0, -0.413, -0.1671],
        [0.35766849176297305, 0.2065, -0.1671], [0.3576684917629732, -0.2065, -0.1671],
        [-0.35766849176297333, 0.2065, -0.1671], [-0.35766849176297305, -0.2065, -0.1671],
    ])  # [m]
    com = np.array([0.03, 0.001, 0.02])  # [m]
    mapping = np.cross(positions - com, [0.0, 0.0, 1.0]).T  # [m]
    limits = np.full(6, 10.0)  # [N]
    return mapping, np.asarray(request) - mapping @ limits, limits


@pytest.mark.parametrize("axes", [np.eye(3)[:2], np.eye(3)[:2][::-1],
                                  np.array([[0.6, 0.8, 0.0], [-0.8, 0.6, 0.0]])])
def test_off_pulsing_sign_repair(axes):
    """Recover the requested physical torque without positive off-pulsing commands."""
    request = np.array([1.946, 0.62, 0.0])  # [N*m]
    mapping, target, limits = dv_case(request)
    forces, angle = run_mapping(mapping, target, axes, limits)
    assert np.all(forces <= 0.0)
    assert np.all(forces >= -limits)
    actual = mapping @ np.clip(limits + forces, 0.0, limits)
    np.testing.assert_allclose(actual, request, atol=1e-11)
    assert angle < 1e-7  # [rad]
    repeat, repeat_angle = run_mapping(mapping, target, axes, limits)
    np.testing.assert_array_equal(repeat, forces)
    assert repeat_angle == angle


def test_off_pulsing_valid_allocation_unchanged():
    """Preserve the previous two-stage result when all of its force signs are valid."""
    mapping, target, limits = dv_case([1.0, -0.5, 0.7])  # [N*m]
    forces, _ = run_mapping(mapping, target, np.eye(3)[:2], limits)
    previous = np.array([0.0, -1.7747382530307778, 0.0, 0.0,
                         -2.1242647119917577, -3.671300055708741])  # [N]
    np.testing.assert_allclose(forces, previous, atol=1e-12)


def test_off_pulsing_sign_repair_saturation_policy():
    """Keep the angle threshold and uniform magnitude scaling after sign repair."""
    mapping, target, limits = dv_case([20.0, -10.0, 0.0])  # [N*m]
    axes = np.eye(3)[:2]
    unscaled, clipped_angle = run_mapping(mapping, target, axes, limits)
    assert np.all(unscaled <= 0.0)
    ratio = np.max(np.abs(unscaled) / limits)
    assert ratio > 1.0
    assert clipped_angle > 1e-3  # [rad]
    scaled, angle = run_mapping(mapping, target, axes, limits, threshold=0.0)  # [rad]
    np.testing.assert_allclose(scaled, unscaled / ratio, atol=1e-12)
    assert np.all(np.abs(scaled) <= limits + 1e-12)  # [N]
    assert angle < 1e-7  # [rad]
    above_threshold, _ = run_mapping(mapping, target, axes, limits, threshold=clipped_angle + 0.01)  # [rad]
    np.testing.assert_array_equal(above_threshold, unscaled)


@pytest.mark.parametrize("negative_map,target", [
    (
        [[-1.1135678027305844, -0.610449764165481, 0.7027921937839988,
          0.5729354476730136, 0.8746261868597082, -1.1565728832115514],
         [-0.4399082506198152, -0.5721698878215957, 0.6310443955363568,
          -1.1695419518018129, 0.6831434865550002, -0.5624630096414787]],  # [m]
        [1.5021167576433025, -1.6574841602170542],  # [N*m]
    ),
    (
        [[0.7568699826829209, 0.987399395290256, 0.595944839090578,
          0.871199046566031, 0.49310690023242065, 0.327992954396479],
         [0.6535654743891809, -0.15824801477565775, 0.8030253724268648,
          0.4909299555562265, -0.8699687264167452, 0.9446801690870139]],  # [m]
        [-0.7635283049704494, 1.1442572211078152],  # [N*m]
    ),
], ids=["requires-reconsidering-discarded-thrusters", "unreachable-torque"])
def test_off_pulsing_constrained_residual(negative_map, target):
    """Match SciPy's optimal residual for feasible and infeasible sign-constrained requests."""
    matrix = np.asarray(negative_map)
    target = np.asarray(target)
    mapping = np.vstack([-matrix, np.zeros(matrix.shape[1])])  # [m]
    forces, _ = run_mapping(mapping, np.append(target, 0.0), np.eye(3)[:2],
                            np.full(matrix.shape[1], 10.0))  # [N]
    _, expected_residual = nnls(matrix, target)
    assert np.all(forces <= 0.0)
    assert np.linalg.norm(matrix @ (-forces) - target) == pytest.approx(expected_residual, abs=1e-10)


@pytest.mark.parametrize("dimension,count", [(2, 6), (3, 6), (2, 36), (3, 36)])
def test_off_pulsing_sign_repair_oracle(dimension, count, capfd):
    """Compare repaired allocations with an independent NNLS oracle across geometries and ranks."""
    rng = np.random.default_rng(1661 + dimension + count)
    checked = 0
    for _ in range(2000):
        matrix = rng.normal(size=(dimension, count))  # [m]
        target = rng.normal(size=dimension)  # [N*m]
        # Select cases that exercise the repair; this is not the expected constrained solution.
        first = np.linalg.lstsq(matrix, target, rcond=None)[0]
        selected = first > 0.0
        if np.linalg.matrix_rank(matrix[:, selected]) < dimension:
            continue
        second = np.linalg.lstsq(matrix[:, selected], target, rcond=None)[0]
        if np.min(second) >= -1e-6:  # [N]
            continue
        mapping = np.zeros((3, count))  # [m]
        mapping[:dimension] = -matrix
        body_target = np.zeros(3)  # [N*m]
        body_target[:dimension] = target
        forces, _ = run_mapping(mapping, body_target, np.eye(3)[:dimension], np.full(count, 10.0))  # [N]
        _, expected_residual = nnls(matrix, target)
        assert np.all(np.isfinite(forces))
        assert np.all(forces <= 0.0)
        assert np.linalg.norm(matrix @ (-forces) - target) == pytest.approx(expected_residual, abs=1e-10)
        checked += 1
        if checked == 12:
            break
    assert checked == 12
    assert "did not converge" not in capfd.readouterr().out


@pytest.mark.parametrize("angle", [0.0, 0.2, 1.0, 1.2])  # [rad]
def test_sign_repair_weak_thruster(angle):
    """Track an achievable torque without concentrating force on a weak, aligned thruster."""
    mapping, _, limits = dv_case([0.0, 0.0, 0.0])  # [N*m]
    mapping = np.ascontiguousarray(mapping)
    target = np.array([2.006, -1.18, 0.0])  # [N*m]
    mapping[:, 2] = -0.001 * target / np.linalg.norm(target)  # [m]
    request = np.array([-0.11761934215157721, 3.901755118895366, 0.0])  # [N*m]
    axes = np.array([[np.cos(angle), np.sin(angle), 0.0], [-np.sin(angle), np.cos(angle), 0.0]])
    forces, _ = run_mapping(mapping, target, axes, limits, threshold=0.0, request=request)  # [rad]
    assert np.all(forces <= 0.0)
    assert np.max(np.abs(forces)) < 3.5  # [N]
    np.testing.assert_allclose(mapping @ np.clip(limits + forces, 0.0, limits), request, atol=1e-11)


@pytest.mark.parametrize("angle", [0.2, 1.0, 1.2])  # [rad]
def test_sign_repair_roundoff_boundary(angle):
    """Clamp roundoff-sized positive forces without changing the allocation or saturation decision."""
    mapping, _, limits = dv_case([0.0, 0.0, 0.0])  # [N*m]
    mapping = np.ascontiguousarray(mapping)
    request = np.array([-8.398216897446336, -3.7203386645332173, 0.0])  # [N*m]
    target = request - mapping @ limits
    baseline, _ = run_mapping(mapping, target, np.eye(3)[:2], limits, threshold=0.0, request=request)  # [rad]
    axes = np.array([[np.cos(angle), np.sin(angle), 0.0], [-np.sin(angle), np.cos(angle), 0.0]])
    forces, _ = run_mapping(mapping, target, axes, limits, threshold=0.0, request=request)  # [rad]
    assert np.all(forces <= 0.0)
    expected = np.array([-10.0, 0.0, 0.0, 0.0, -9.648235743193, 0.0])  # [N]
    np.testing.assert_allclose(baseline, expected, atol=1e-11)
    np.testing.assert_allclose(forces, baseline, atol=1e-11)


@pytest.mark.parametrize("angle", [0.0, 0.2, 0.7, 1.2])  # [rad]
def test_sign_repair_proportional_columns(angle):
    """Obtain one minimum-norm allocation for proportional columns under equivalent control bases."""
    mapping, _, limits = dv_case([0.0, 0.0, 0.0])  # [N*m]
    mapping = np.ascontiguousarray(mapping)
    mapping[:, 1] = 3.7 * mapping[:, 3]
    limits[[1, 3]] = 0.1  # [N]
    target = np.array([0.657585803989712, -1.0573329814271553, 0.0])  # [N*m]
    request = target + mapping @ limits
    unscaled, _ = run_mapping(mapping, target, np.eye(3)[:2], limits, request=request)
    np.testing.assert_allclose(mapping @ (limits + unscaled), request, atol=1e-11)
    expected = unscaled / max(1.0, np.max(np.abs(unscaled) / limits))
    baseline, _ = run_mapping(mapping, target, np.eye(3)[:2], limits, threshold=0.0, request=request)  # [rad]
    axes = np.array([[np.cos(angle), np.sin(angle), 0.0], [-np.sin(angle), np.cos(angle), 0.0]])
    forces, _ = run_mapping(mapping, target, axes, limits, threshold=0.0, request=request)  # [rad]
    assert np.all(forces <= 0.0)
    assert np.all(np.abs(forces) <= limits)
    np.testing.assert_allclose(forces, expected, atol=1e-11)
    np.testing.assert_allclose(forces, baseline, atol=1e-11)
    np.testing.assert_allclose(forces[1], 3.7 * forces[3], atol=1e-11)
    np.testing.assert_allclose(mapping @ (limits + forces), mapping @ (limits + baseline), atol=1e-11)


@pytest.mark.parametrize("dimension,count", [(2, 6), (3, 6), (2, 36), (3, 36)])
def test_sign_repair_minimum_norm(dimension, count):
    """Check the secondary force objective with an independent constrained optimizer."""
    rng = np.random.default_rng(16610 + dimension + count)
    checked = 0
    for _ in range(2000):
        matrix = rng.normal(size=(dimension, count))  # [m]
        target = rng.normal(size=dimension)  # [N*m]
        first = np.linalg.lstsq(matrix, target, rcond=None)[0]
        selected = first > 0.0
        if np.linalg.matrix_rank(matrix[:, selected]) < dimension:
            continue
        second = np.linalg.lstsq(matrix[:, selected], target, rcond=None)[0]
        if np.min(second) >= -1e-6:  # [N]
            continue
        initial, _ = nnls(matrix, target)
        achieved = matrix @ initial
        oracle = minimize(lambda x: 0.5 * np.dot(x, x), initial, jac=lambda x: x,
                          constraints={"type": "eq", "fun": lambda x: matrix @ x - achieved,
                                       "jac": lambda x: matrix},
                          bounds=[(0.0, None)] * count, method="SLSQP",
                          options={"ftol": 1e-12, "maxiter": 200})
        assert oracle.success, oracle.message
        mapping = np.zeros((3, count))  # [m]
        mapping[:dimension] = -matrix
        body_target = np.zeros(3)  # [N*m]
        body_target[:dimension] = target
        forces, _ = run_mapping(mapping, body_target, np.eye(3)[:dimension], np.full(count, 10.0))  # [N]
        np.testing.assert_allclose(-forces, oracle.x, atol=1e-8)
        checked += 1
        if checked == 8:
            break
    assert checked == 8


def test_sign_repair_nearly_dependent_axes(capfd):
    """Preserve the achieved torque when the minimum-force QR has nearly dependent columns."""
    matrix = np.array([
        [-1.4130256338379743, 1.2304149324795781, -0.043652332329482284,
         0.043010572944318974, -0.8385421066037801, 1.256291611959465],
        [0.1004101538355345, 1.2085020934354258, 0.9627449666952517,
         0.8201321008550562, -1.126749577892663, 0.6519236233612349],
        [-1.4130259306678625, 1.2304203781036798, -0.043640136325598704,
         0.04298679015671169, -0.8385381946136046, 1.2562983566668235],
    ])  # [m]
    target = np.array([-0.6488054473181674, 0.07208248387471818, 0.11832321311352825])  # [N*m]
    forces, _ = run_mapping(-matrix, target, np.eye(3), np.full(6, 10.0))  # [N]
    assert np.all(forces <= 0.0)
    _, expected_residual = nnls(matrix, target)
    assert np.linalg.norm(matrix @ (-forces) - target) == pytest.approx(expected_residual, abs=1e-9)  # [N*m]
    assert "did not converge" not in capfd.readouterr().out


@pytest.mark.parametrize("basis", ["identity", "rotated", "permuted"])
def test_sign_repair_feasible_boundary(basis, capfd):
    """Converge to the minimum-force allocation on a cone boundary under equivalent control bases."""
    matrix = np.array([
        [
            1.059027507101065, -0.9061780567391545, -0.33784478986673827, 1.0633584774999423,
            0.9435782158506654, 0.6007305742555636, -1.6338958656404305, -0.6447083001311033,
            0.970482357841659, -1.1119572837994347, 0.5495566188131048, 1.3160562967834175,
            0.017222759532981906, -1.5435116034185627, -1.8425022448196469, 0.7041948806152312,
            -0.7703853338201869, 0.20725075318116984, -1.3014489537518257, 0.00410844201824772,
            0.06342988750507063, -0.7403659281158405, -1.3242777886611357, -1.6197735985162582,
            -0.9641404656607894, -3.0095203329025866, -1.6362000032919166, -1.5417243583202862,
            -0.9818945741259133, -0.01237499914482526, 0.9566355013531577, -1.379279263167336,
            -1.9407159838049595, -0.9143651308273815, -0.5566717155407187, -0.369538713935465,
        ],
        [
            0.3382713181095196, -0.520767861124866, 0.029688438117407526, 0.5817150678618852,
            0.36169713388686064, 0.560414545225688, -1.4012706698031554, -0.45448681849149275,
            0.3784411633805196, -0.9718473275568373, 0.7765537610067668, 1.346560439608563,
            0.9151226779607745, -1.3399988223217627, 0.7353123626466909, 0.34538728834363897,
            0.2691287700261202, 0.7744817220815985, 0.5636260745768895, -0.06294619292617891,
            1.7736552814276272, 1.865041275013689, -0.2932494059518405, 0.009597358915459533,
            0.22616511413876053, 0.6073776823532903, -0.8105764237313827, 0.23151887865513565,
            1.0421341853115555, 1.5135580236679216, 1.3394420599326988, -1.060256733769729,
            1.2965515749562648, 0.6766891494015277, 0.2209063001039795, 1.3241801645039533,
        ],
        [
            -1.3985760137894137, 0.4403158771439382, 0.8961172947456196, -0.6127689646421799,
            -1.0489252766089747, 0.41174149939464283, -0.7177622347493754, 0.03864926853552625,
            -1.057804087475578, -0.5480050374459972, 1.239538753337058, 1.2905852468810675,
            -1.615214979657993, -1.5239252791201308, -0.16820220789765009, -1.2343250517942062,
            0.3366139497367615, 0.3430237553150128, 1.3522118625391306, -1.0297029390228896,
            2.147818229667914, 1.4086758889366415, 0.4918370264822884, -1.926810910409885,
            0.02879132629143523, 1.9159540403237012, 1.2119662351063867, -0.19454781653429246,
            -1.25368358528686, 0.6031006099168131, -0.27633800246942386, -0.32211778465823404,
            -2.106126158579355, 0.05649896983567405, 0.5610183444656046, -1.123756457320214,
        ],
    ])  # [m]
    target = np.array([
        -0.763512023129114, 0.5010027058329709, 3.4440404596229266,
    ])  # [N*m]
    rotated_axes = np.array([
        [
            -0.30968844813965757, 0.08916492538015479, -0.9466481295448704,
        ],
        [
            -0.9508086266372431, -0.03687715238458441, 0.3075760574950588,
        ],
        [
            -0.00748469111253888, 0.9953339598918323, 0.09619920833870887,
        ],
    ])  # [-]
    limits = np.array([
        1.7858238813588225, 1.9802012482083258, 1.3059213434128072, 1.878078930681193,
        1.8046490797207784, 1.3163243152350137, 1.2423574686508245, 0.6540492859396375,
        1.3455305038651866, 1.7774736097683608, 0.9257479165967167, 1.9056686175450908,
        1.8333772173307237, 1.3834988068457448, 0.1554595404528163, 0.779372718031724,
        1.6240373156258285, 0.47242745318570645, 0.17737073428521105, 1.8195720558510595,
        0.693594391575096, 1.8298350217959731, 0.3117747671287628, 1.8067016605959096,
        0.934372292746934, 1.5405640986186189, 1.0819946387128045, 0.5223864955215068,
        0.14184318684007002, 0.867121183314106, 1.7883729861530908, 1.568248171218511,
        1.0750394051203267, 0.6809501829527747, 0.9528841137840521, 0.983470910764787,
    ])  # [N]
    axes = {"identity": np.eye(3), "rotated": rotated_axes, "permuted": np.eye(3)[[2, 0, 1]]}[basis]

    # The first 12 columns span the boundary face.  Other columns point strictly to one side;
    # they cannot contribute to this target with nonnegative coefficients.
    left, _, _ = np.linalg.svd(matrix[:, :12], full_matrices=False)
    face = left[:, :2].T @ matrix[:, :12]
    face_target = left[:, :2].T @ target
    initial, _ = nnls(face, face_target)
    oracle = minimize(lambda x: 0.5 * np.dot(x, x), initial, jac=lambda x: x,
                      constraints={"type": "eq", "fun": lambda x: face @ x - face_target,
                                   "jac": lambda x: face},
                      bounds=[(0.0, None)] * 12, method="SLSQP",
                      options={"ftol": 1e-12, "maxiter": 200})
    assert oracle.success, oracle.message
    expected = np.zeros(36)  # [N]
    expected[:12] = -oracle.x
    forces, _ = run_mapping(-matrix, target, axes, limits)
    np.testing.assert_allclose(forces, expected, rtol=0.0, atol=1e-10)  # [N]
    assert np.all(forces <= 0.0)
    np.testing.assert_allclose(matrix @ (-forces), target, rtol=0.0, atol=1e-10)  # [N*m]

    # Preserve the existing uniform scaling once the clipped torque exceeds the angle threshold.
    scale = np.max(np.abs(expected) / limits)
    assert scale > 1.0
    scaled, _ = run_mapping(-matrix, target, axes, limits, threshold=0.0)  # [rad]
    np.testing.assert_allclose(scaled, expected / scale, rtol=0.0, atol=1e-10)  # [N]
    np.testing.assert_allclose(-matrix @ np.clip(limits + scaled, 0.0, limits),
                               -matrix @ (limits + expected / scale), rtol=0.0, atol=1e-10)  # [N*m]
    assert "did not converge" not in capfd.readouterr().out


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
