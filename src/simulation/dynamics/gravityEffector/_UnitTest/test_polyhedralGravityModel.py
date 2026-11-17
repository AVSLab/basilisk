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

"""Validate polyhedral gravity normalization, initialization, and energy.

A closed C-shaped body contains fourteen equal cubes. Its signed volume is
14 km^3, whereas summing the absolute tetrahedral volumes gives 18 km^3.
Acceleration and potential are checked against the superposition of separate
cube models and their point-mass limits. Stored Eros results check compatibility
with a mesh whose tetrahedral contributions are all positive (issue 1681).
Potential gradients, inertial-frame evaluations, and spacecraft energy checks
cover the sign and reference-frame corrections in issue 1689.
"""

from collections import Counter
from types import SimpleNamespace

import numpy as np
import pytest

from Basilisk.architecture.bskLogging import BasiliskError
from Basilisk.architecture import messaging
from Basilisk.simulation import gravityEffector, spacecraft, stateArchitecture, svIntegrators
from Basilisk.simulation.gravityModel import GravityModel
from Basilisk.simulation.polyhedralGravityModel import PolyhedralGravityModel
from Basilisk.simulation.pointMassGravityModel import PointMassGravityModel
from Basilisk.simulation.sphericalHarmonicsGravityModel import SphericalHarmonicsGravityModel
from Basilisk.utilities import SimulationBaseClass, macros
from Basilisk.utilities.pythonVariableLogger import PythonVariableLogger
from Basilisk.utilities.supportDataTools.dataFetcher import DataFile, get_path


SIDE_LENGTH = 1000.0  # [m]
MU = 4.46e5  # [m^3/s^2]


def _make_mesh(cells):
    """Triangulate the boundary of occupied grid cells about their center of mass.

    :param cells: Set of integer grid coordinates for equal, uniform cubes.
    :return: Vertices in meters, zero-based facets, and cube centers in meters.
    """
    # Outward-oriented cube faces in unit grid coordinates.
    quads = {
        (1, 0, 0): [(1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1)],
        (-1, 0, 0): [(0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0)],
        (0, 1, 0): [(0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0)],
        (0, -1, 0): [(0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1)],
        (0, 0, 1): [(0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1)],
        (0, 0, -1): [(0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0)],
    }
    vertices, indices, facets = [], {}, []
    for cell in sorted(cells):
        for direction, quad in quads.items():
            if tuple(np.add(cell, direction)) in cells:
                continue
            facet = []
            for vertex in quad:
                point = tuple(np.add(cell, vertex))
                if point not in indices:
                    indices[point] = len(vertices)
                    vertices.append(point)
                facet.append(indices[point])
            facets.extend([[facet[0], facet[1], facet[2]], [facet[0], facet[2], facet[3]]])

    # Every edge must be shared by two triangles traversing it in opposite directions.
    edges = Counter(
        (start, end) for facet in facets for start, end in zip(facet, np.roll(facet, -1))
    )
    assert all(count == 1 and edges[(end, start)] == 1 for (start, end), count in edges.items())
    centers = np.array(sorted(cells), dtype=float) + 0.5
    center_of_mass = centers.mean(axis=0)
    return (
        (np.array(vertices, dtype=float) - center_of_mass) * SIDE_LENGTH,
        np.array(facets),
        (centers - center_of_mass) * SIDE_LENGTH,
    )


def _make_model(vertices, facets, mu=MU):
    """Configure a model without initializing it.

    :param vertices: Vertex positions in meters.
    :param facets: Zero-based vertex indices for each triangular facet.
    :param mu: Gravitational parameter in m^3/s^2.
    :return: Configured polyhedral gravity model.
    """
    model = PolyhedralGravityModel()
    model.xyzVertex = vertices.tolist()
    model.orderFacet = (facets + 1).tolist()
    model.muBody = mu
    return model


def _make_invalid_mesh(invalid_volume):
    """Return a closed mesh with an invalid signed volume.

    :param invalid_volume: Mesh modification producing a non-positive or non-finite volume.
    :return: Vertices in meters and zero-based triangular facets.
    """
    vertices, facets, _ = _make_mesh({(0, 0, 0)})
    if invalid_volume == "reversed":
        facets = facets[:, ::-1]
    elif invalid_volume == "zero":
        # A flat tetrahedral surface has zero volume but no zero-area triangles.
        vertices = SIDE_LENGTH * np.array([[-1., -1., 0.], [1., -1., 0.], [1., 1., 0.], [-1., 1., 0.]])
        facets = np.array([[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]])
    else:
        vertices[0, 0] = {"nan": np.nan, "infinite": np.inf, "negative_infinite": -np.inf}[invalid_volume]
    return vertices, facets


@pytest.fixture(scope="module")
def c_shape():
    """Return an initialized C-shaped body and its constituent cube centers."""
    cells = {(x, y, 0) for x in (0, 1) for y in range(5)}
    cells.update({(2, 0, 0), (3, 0, 0), (2, 4, 0), (3, 4, 0)})
    vertices, facets, centers = _make_mesh(cells)
    tetrahedra = np.einsum(
        "ij,ij->i", vertices[facets[:, 0]],
        np.cross(vertices[facets[:, 1]], vertices[facets[:, 2]]),
    ) / 6.0
    assert tetrahedra.sum() == pytest.approx(14 * SIDE_LENGTH**3)
    assert np.abs(tetrahedra).sum() == pytest.approx(18 * SIDE_LENGTH**3)
    assert np.count_nonzero(tetrahedra < 0.0) == 8
    model = _make_model(vertices, facets)
    assert model.initializeParameters() is None
    return model, centers


@pytest.mark.parametrize(
    "position", [[6000.0, 3000.0, 2000.0], [1200.0, 0.0, 100.0], [-4000.0, 2000.0, -3000.0]]  # [m]
)
def test_concave_body_matches_cube_superposition(c_shape, position):
    """Check acceleration and potential against fourteen correctly normalized cubes.

    The second evaluation point lies in the opening of the C-shaped body.

    :param c_shape: Initialized C-shaped model and its cube centers.
    :param position: Evaluation point in meters relative to the center of mass.
    """
    model, centers = c_shape
    vertices, facets, _ = _make_mesh({(0, 0, 0)})
    cube = _make_model(vertices, facets, MU / len(centers))
    assert cube.initializeParameters() is None
    expected_acceleration = np.zeros(3)
    expected_potential = 0.0  # [m^2/s^2]
    for center in centers:
        relative_position = (np.array(position) - center).tolist()
        expected_acceleration += np.asarray(cube.computeField(relative_position)).ravel()
        expected_potential += cube.computePotentialEnergy(relative_position)

    acceleration = np.asarray(model.computeField(position)).ravel()
    # Compare vector norms because symmetry makes one component vanish in the opening.
    relative_error = np.linalg.norm(acceleration - expected_acceleration) / np.linalg.norm(expected_acceleration)
    assert relative_error < 1e-11
    assert model.computePotentialEnergy(position) == pytest.approx(expected_potential, rel=1e-11)


@pytest.mark.parametrize("direction", [[1.0, 0.3, 0.2], [0.1, -1.0, 0.4], [0.2, 0.1, -1.0]])
def test_concave_body_far_field(c_shape, direction):
    """Check the vector acceleration and potential against the point-mass limits.

    :param c_shape: Initialized C-shaped model and its cube centers.
    :param direction: Dimensionless direction of the evaluation point.
    """
    model, _ = c_shape
    distance = 5.0e6  # [m]
    position = np.asarray(direction) / np.linalg.norm(direction) * distance
    expected_acceleration = -MU * position / distance**3
    acceleration = np.asarray(model.computeField(position.tolist())).ravel()
    # Allow finite-size corrections and cancellation in the distant polyhedral field.
    relative_error = np.linalg.norm(acceleration - expected_acceleration) / np.linalg.norm(expected_acceleration)
    assert relative_error < 2e-5
    assert model.computePotentialEnergy(position.tolist()) == pytest.approx(-MU / distance, rel=2e-5)


@pytest.mark.parametrize("invalid_volume", ["reversed", "zero", "nan", "infinite", "negative_infinite"])
@pytest.mark.parametrize("through_body", [False, True], ids=["direct", "body"])
def test_invalid_volume_is_rejected(invalid_volume, through_body):
    """Reject invalid volume directly and through the body initialization error path.

    :param invalid_volume: Mesh modification producing a non-positive or non-finite volume.
    :param through_body: Whether to initialize through ``GravBodyData``.
    """
    vertices, facets = _make_invalid_mesh(invalid_volume)
    model = _make_model(vertices, facets)
    message = "signed volume must be finite and positive"
    if through_body:
        body = gravityEffector.GravBodyData()
        body.planetName = "invalid_polyhedron"
        body.mu = MU
        body.gravityModel = model
        with pytest.raises(BasiliskError, match=message):
            body.initBody(0)
    else:
        error = model.initializeParameters()
        assert isinstance(error, str)
        assert message in error


@pytest.mark.parametrize("invalid_volume", ["reversed", "zero", "nan", "infinite", "negative_infinite"])
@pytest.mark.parametrize("through_body", [False, True], ids=["direct", "body"])
@pytest.mark.parametrize("previously_initialized", [False, True], ids=["new", "initialized"])
def test_initialization_retry_matches_fresh_model(invalid_volume, through_body, previously_initialized):
    """Recover from a rejected mesh without retaining facet or edge dyads.

    After rejection, replace the mesh with a valid cube and compare acceleration
    and potential against a fresh model. An optional earlier successful
    initialization also populates the edge cache, which a rejected mesh does not
    rebuild. Cyclically permuting the earlier vertices changes the cached dyads
    associated with each index while preserving a valid, outward-oriented cube.

    :param invalid_volume: Rejected mesh configuration.
    :param through_body: Whether all initialization attempts use ``GravBodyData``.
    :param previously_initialized: Whether to initialize a valid mesh before rejection.
    """
    vertices, facets, _ = _make_mesh({(0, 0, 0)})
    model = _make_model(np.roll(vertices, 1, axis=1), facets)
    if through_body:
        body = gravityEffector.GravBodyData()
        body.planetName = "retry_polyhedron"
        body.mu = MU
        body.gravityModel = model

        def initialize():
            """Initialize the same gravity body on every attempt."""
            return body.initBody(0)
    else:
        initialize = model.initializeParameters

    if previously_initialized:
        assert initialize() is None

    invalid_vertices, invalid_facets = _make_invalid_mesh(invalid_volume)
    model.xyzVertex = invalid_vertices.tolist()
    model.orderFacet = (invalid_facets + 1).tolist()
    message = "signed volume must be finite and positive"
    if through_body:
        with pytest.raises(BasiliskError, match=message):
            initialize()
    else:
        error = initialize()
        assert isinstance(error, str)
        assert message in error

    model.xyzVertex = vertices.tolist()
    model.orderFacet = (facets + 1).tolist()
    assert initialize() is None
    fresh_model = _make_model(vertices, facets)
    assert fresh_model.initializeParameters() is None
    positions = [[6000.0, 3000.0, 2000.0], [-4000.0, 2000.0, -3000.0]]  # [m]
    for position in positions:
        acceleration = np.asarray(model.computeField(position)).ravel()
        potential = model.computePotentialEnergy(position)
        assert np.isfinite(acceleration).all()
        assert np.isfinite(potential)
        np.testing.assert_allclose(
            acceleration, np.asarray(fresh_model.computeField(position)).ravel(), rtol=1e-13, atol=0.0
        )
        assert potential == pytest.approx(fresh_model.computePotentialEnergy(position), rel=1e-13)


def test_eros_field_and_potential_magnitude_are_unchanged():
    """Preserve the Eros field and potential magnitude while correcting its sign."""
    model = PolyhedralGravityModel().loadFromFile(str(get_path(DataFile.LocalGravData.eros007790)))
    model.muBody = 4.46275472004e5  # [m^3/s^2]
    assert model.initializeParameters() is None
    positions = [[40000.0, 30000.0, 10000.0], [-30000.0, 20000.0, -15000.0]]  # [m]
    # Reference outputs from develop e5eddcb99d; negate the old positive potentials.
    accelerations = [
        [-0.0001326573167994466, -0.0001060227211785598, -3.476412823086498e-5],
        [0.0002260472476788, -0.00016664929978675686, 0.00012864272573401533],
    ]  # [m/s^2]
    potentials = [-8.782543599089866, -11.646507538262664]  # [m^2/s^2]
    for position, acceleration, potential in zip(positions, accelerations, potentials):
        np.testing.assert_allclose(
            np.asarray(model.computeField(position)).ravel(), acceleration, rtol=1e-11, atol=0.0
        )
        assert model.computePotentialEnergy(position) == pytest.approx(potential, rel=1e-11)


def test_base_potential_api_preserves_position_keyword(c_shape):
    """Preserve base-class positional and keyword calls for a polyhedral model.

    :param c_shape: Initialized concave model and its cube centers.
    """
    model, _ = c_shape
    position = [6000.0, 3000.0, 2000.0]  # [m] Body-fixed position despite the historical keyword suffix.
    expected = model.computePotentialEnergy(position)
    assert GravityModel.computePotentialEnergy(model, position) == pytest.approx(expected, rel=1e-14)
    assert GravityModel.computePotentialEnergy(model, positionWrtPlanet_N=position) == pytest.approx(expected, rel=1e-14)


@pytest.mark.parametrize(
    "position", [[6000.0, 3000.0, 2000.0], [1200.0, 0.0, 100.0], [-4000.0, 2000.0, -3000.0]]  # [m]
)
def test_acceleration_is_negative_potential_gradient(c_shape, position):
    """Check the energy sign independently of another polyhedral potential call.

    :param c_shape: Initialized concave model and its cube centers.
    :param position: Exterior evaluation point in body-fixed coordinates [m].
    """
    model, _ = c_shape
    step = 0.1  # [m]
    gradient = np.empty(3)  # [m/s^2]
    for axis in range(3):
        delta = np.eye(3)[axis] * step
        gradient[axis] = (
            model.computePotentialEnergy((np.array(position) + delta).tolist())
            - model.computePotentialEnergy((np.array(position) - delta).tolist())
        ) / (2.0 * step)
    acceleration = np.asarray(model.computeField(position)).ravel()
    relative_error = np.linalg.norm(acceleration + gradient) / np.linalg.norm(acceleration)
    assert relative_error < 1e-7


def _body_rotation(angle):
    """Return the inertial-to-body rotation for a spin about the inertial z axis.

    :param angle: Body rotation angle [rad].
    :return: Direction cosine matrix [-].
    """
    cosine, sine = np.cos(angle), np.sin(angle)
    return np.array([[cosine, sine, 0.0], [-sine, cosine, 0.0], [0.0, 0.0, 1.0]])


@pytest.mark.parametrize("orientation", ["rotating", "zero", "unlinked"])
@pytest.mark.parametrize("offset", [-40.0, 0.0, 70.0])  # [s]
def test_inertial_potential_uses_requested_orientation(c_shape, orientation, offset):
    """Use the message epoch and identity fallback independently of cached field properties.

    :param c_shape: Initialized concave model and its cube centers.
    :param orientation: Rotating message, zero DCM, or unlinked message.
    :param offset: Signed evaluation offset from the message epoch [s].
    """
    model, _ = c_shape
    body = gravityEffector.GravBodyData()
    body.planetName = "energy_test"
    body.mu = MU
    body.gravityModel = model
    body.initBody(0)
    angle = 0.4  # [rad]
    spin_rate = 0.002  # [rad/s]
    epoch = 100.0  # [s]
    payload = messaging.SpicePlanetStateMsgPayload()
    if orientation == "rotating":
        payload.J20002Pfix = _body_rotation(angle).tolist()
        spin_tilde = np.array([[0.0, -spin_rate, 0.0], [spin_rate, 0.0, 0.0], [0.0, 0.0, 0.0]])  # [1/s]
        payload.J20002Pfix_dot = (-spin_tilde @ _body_rotation(angle)).tolist()
    message = messaging.SpicePlanetStateMsg().write(payload, macros.sec2nano(epoch))
    if orientation != "unlinked":
        body.planetBodyInMsg.subscribeTo(message)
    body.loadEphemeris()
    position = np.array([6000.0, 3000.0, 2000.0])  # [m]
    dcm_PN = _body_rotation(angle + spin_rate * offset) if orientation == "rotating" else np.eye(3)
    expected = model.computePotentialEnergy((dcm_PN @ position).tolist())
    evaluation_time = macros.sec2nano(epoch + offset)  # [ns]
    # Potential evaluation must also work before state-engine properties are registered.
    assert body.computePotentialEnergyInertial(position.tolist(), evaluation_time) == pytest.approx(
        expected, rel=1e-13
    )
    manager = stateArchitecture.DynParamManager()
    body.registerProperties(manager)
    body.computeGravityInertial(position.tolist(), macros.sec2nano(epoch))
    assert body.computePotentialEnergyInertial(position.tolist(), evaluation_time) == pytest.approx(
        expected, rel=1e-13
    )
    expected_acceleration = dcm_PN.T @ np.asarray(model.computeField((dcm_PN @ position).tolist())).ravel()
    np.testing.assert_allclose(
        np.asarray(body.computeGravityInertial(position.tolist(), evaluation_time)).ravel(),
        expected_acceleration, rtol=1e-13, atol=0.0,
    )


def _make_energy_simulation(model_kind, frame, integrator_name, spin_rate, moving_body, time_step):
    """Configure a spacecraft and an analytic, once-written planet ephemeris.

    :param model_kind: Polyhedral, point-mass, or degree-zero spherical-harmonic model.
    :param frame: Central-body, inertial, or secondary-body configuration.
    :param integrator_name: Name of the integrator class.
    :param spin_rate: Constant planet spin about the inertial z axis [rad/s].
    :param moving_body: Whether to translate the planet with a constant velocity.
    :param time_step: Simulation task period [s].
    :return: Simulation, model, ephemeris, and logger handles.
    """
    if model_kind == "polyhedral":
        vertices, facets, _ = _make_mesh({(0, 0, 0), (1, 0, 0), (2, 0, 0)})
        model = _make_model(vertices, facets)
    elif model_kind == "point_mass":
        model = PointMassGravityModel()
    else:
        model = SphericalHarmonicsGravityModel()
        model.maxDeg = 0
        model.cBar = [[1.0]]  # [-]
        model.sBar = [[0.0]]
        model.radEquator = SIDE_LENGTH
    body = gravityEffector.GravBodyData()
    body.planetName = "energy_body"
    body.isCentralBody = frame == "central"
    body.mu = MU
    body.gravityModel = model
    angle = 0.4  # [rad]
    payload = messaging.SpicePlanetStateMsgPayload()
    payload.J20002Pfix = _body_rotation(angle).tolist()
    spin_tilde = np.array([[0.0, -spin_rate, 0.0], [spin_rate, 0.0, 0.0], [0.0, 0.0, 0.0]])  # [1/s]
    payload.J20002Pfix_dot = (-spin_tilde @ _body_rotation(angle)).tolist()
    if moving_body or frame == "secondary":
        payload.PositionVector = [8000.0, 1000.0, -500.0]  # [m]
    if moving_body:
        payload.VelocityVector = [0.1, -0.2, 0.05]  # [m/s]
    message = messaging.SpicePlanetStateMsg().write(payload)
    body.planetBodyInMsg.subscribeTo(message)
    bodies = [body]
    if frame == "secondary":
        central = gravityEffector.GravBodyData()
        central.planetName = "central"
        central.isCentralBody = True
        central.mu = 1e5  # [m^3/s^2]
        bodies.append(central)

    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    process.addTask(sim.CreateNewTask("task", macros.sec2nano(time_step)))
    sc = spacecraft.Spacecraft()
    sc.ModelTag = "energy_spacecraft"
    sc.hub.mHub = 75.0  # [kg]
    sc.hub.IHubPntBc_B = np.diag([40.0, 50.0, 60.0]).tolist()  # [kg*m^2]
    position = np.array([6000.0, 3000.0, 1000.0])  # [m]
    velocity = np.array([-2.0, 6.0, 0.4])  # [m/s]
    if frame != "central":
        position += np.array(payload.PositionVector)
        velocity += np.array(payload.VelocityVector)
    sc.hub.r_CN_NInit = position.tolist()
    sc.hub.v_CN_NInit = velocity.tolist()
    sc.gravField.gravBodies = spacecraft.GravBodyVector(bodies)
    integrator = getattr(svIntegrators, integrator_name)(sc)
    sc.setIntegrator(integrator)
    sim.AddModelToTask("task", sc)
    # Record the integrated coordinates directly; message coordinates also include
    # extrapolation of a moving central body's ephemeris.
    log = PythonVariableLogger({
        "position": lambda _: sc.dynManager.getStateObject(sc.hub.nameOfHubPosition).getState(),
        "velocity": lambda _: sc.dynManager.getStateObject(sc.hub.nameOfHubVelocity).getState(),
        "energy": lambda _: sc.totOrbEnergy,
    })
    sim.AddModelToTask("task", log)
    sim.InitializeSimulation()
    return SimpleNamespace(
        sim=sim, sc=sc, body=body, model=model, bodies=bodies, log=log,
        message=message, payload=payload, angle=angle, spin_rate=spin_rate, integrator=integrator,
    )


@pytest.mark.parametrize("model_kind", ["polyhedral", "point_mass", "spherical_harmonics"])
@pytest.mark.parametrize("frame", ["central", "inertial", "secondary"])
@pytest.mark.parametrize("integrator_name", ["svIntegratorRK4", "svIntegratorRKF45"])
def test_reported_energy_uses_body_frame_and_output_epoch(model_kind, frame, integrator_name):
    """Check energy at the output epoch for moving, rotating gravity bodies.

    RKF45 evaluates its last stage before the step endpoint. Energy must use the
    output time for both planet translation and orientation, rather than that stage.
    The secondary-body case exercises both existing potential contribution terms.

    :param model_kind: Gravity model whose energy contribution is checked.
    :param frame: Integration origin and role of the tested body.
    :param integrator_name: Fixed-step or adaptive integrator.
    """
    spin_rate = 0.002  # [rad/s]
    time_step = 10.0  # [s]
    result = _make_energy_simulation(model_kind, frame, integrator_name, spin_rate, True, time_step)
    duration = 120.0  # [s]
    result.sim.ConfigureStopTime(macros.sec2nano(duration))
    result.sim.ExecuteSimulation()
    for time, position, velocity, energy in zip(
        result.log.times()[1:] * macros.NANO2SEC,
        result.log.position[1:], result.log.velocity[1:], result.log.energy[1:],
    ):
        body_position = np.array(result.payload.PositionVector) + time * np.array(result.payload.VelocityVector)
        relative_position = position if frame == "central" else position - body_position
        dcm_PN = _body_rotation(result.angle + spin_rate * time)
        potential = result.model.computePotentialEnergy((dcm_PN @ relative_position).tolist())
        if frame == "secondary":
            potential += result.model.computePotentialEnergy((dcm_PN @ body_position).tolist())
            potential += result.bodies[1].gravityModel.computePotentialEnergy(position.tolist())
        expected_energy = result.sc.hub.mHub * (0.5 * velocity.dot(velocity) + potential)
        absolute_tolerance = 1e-8  # [J]
        assert energy == pytest.approx(expected_energy, rel=1e-12, abs=absolute_tolerance)


@pytest.mark.parametrize("spin_rate", [0.0, 0.0004])  # [rad/s]
def test_polyhedral_energy_or_jacobi_is_conserved(spin_rate):
    """Conserve energy for a static body and the Jacobi integral for uniform rotation.

    :param spin_rate: Constant planet rotation rate [rad/s].
    """
    time_step = 1.0  # [s]
    result = _make_energy_simulation("polyhedral", "central", "svIntegratorRK4", spin_rate, False, time_step)
    duration = 3600.0  # [s]
    result.sim.ConfigureStopTime(macros.sec2nano(duration))
    result.sim.ExecuteSimulation()
    specific_energy = result.log.energy[1:] / result.sc.hub.mHub
    angular_momentum = np.cross(result.log.position[1:], result.log.velocity[1:])[:, 2]
    jacobi = specific_energy - spin_rate * angular_momentum
    assert np.ptp(jacobi) / abs(jacobi[0]) < 1e-9
    if spin_rate:
        minimum_energy_variation = 1e-3  # [m^2/s^2]
        assert np.ptp(specific_energy) > minimum_energy_variation


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
