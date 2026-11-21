Executive Summary
-----------------

This module searches a grid of spacecraft attitudes for a path subject to keep-in and keep-out
pointing cones. It fits a spline to the selected waypoints and publishes an attitude reference
for a tracking controller. A fitted reference can cross a cone boundary even when every
waypoint satisfies the pointing constraints.


Message Connection Descriptions
-------------------------------
The following diagram and table list all the module input and output messages.  The module message connection is
set by the user from Python.  The message type contains a link to the message structure definition, while the
description provides information on what this message is used for.

.. bsk-module-io:: constrainedAttitudeManeuver
    :caption: Module I/O Messages

    output attRefOutMsg AttRefMsgPayload
        Output Attitude Reference Message.
    output attRefOutMsgC AttRefMsgPayload
        C-wrapped Output Attitude Reference Message.
    input scStateInMsg SCStatesMsgPayload
        Required spacecraft state. Its source must have written a message before reset.
    input vehicleConfigInMsg VehicleConfigMsgPayload
        Required vehicle configuration supplying the inertia tensor. Its source must have written a message before reset.
    input keepOutCelBodyInMsg SpicePlanetStateMsgPayload
        Optional keep-out planet state.
    input keepInCelBodyInMsg SpicePlanetStateMsgPayload
        Optional keep-in planet state.

Module Assumptions and Limitations
----------------------------------
The module, in its current form, allows to only include one keep-out and one keep-in celestial object(s). Multiple boresights (up to 10) can be specified as either
keep-in or keep-out body directions. Each category has ten slots. Appending beyond that limit raises an error.
Both append methods normalize finite, nonzero body-frame directions. Cone half-angles
are measured from the boresight and must be in :math:`[0, \pi]` radians, including both endpoints.
Both cone types include their rim within angular roundoff. A point on the rim satisfies
a keep-in constraint and violates a keep-out constraint.
When setting the public boresight arrays directly, supply unit vectors. The planner
checks the active entries when reading inputs and when building the grid.
The keep-out constraint is considered respected when all the keep-out body-fixed boresights are outside of the keep-out zone
simultaneously. The keep-in constraint is satisfied when at least one keep-in body-fixed
direction is inside the keep-in zone. Each category is active only when it has a boresight
and its celestial-body message is linked. An active category requires a written message at reset
and a finite, nonzero separation between the spacecraft and the celestial body.
Inertial cone axes supplied directly must also be finite and nonzero.

``GenerateGrid()`` reclassifies both endpoint attitudes using the current boresights and the
last-read celestial-body directions. Call ``ReadInputs()`` before a direct rebuild after enabling
or disabling a category, after replacing a source even if it stays linked, or to use updated
spacecraft or celestial-body positions. Reading inputs invalidates the previous grid and reference.
``effortBasedAStar()``, ``spline()``, and ``effortEvaluation()`` use cached spacecraft inputs
and require a successful ``ReadInputs()``. If a read fails, correct the input and read again.
``Reset()`` refreshes the inputs automatically.

A target attitude that violates a pointing constraint causes planning to fail. An initially
noncompliant attitude raises a warning so the planner can search for a path out of the violation.
The remaining waypoints satisfy the cones evaluated with the directions read at reset.
The User Guide describes how these checks apply to the fitted reference.

The supported grid refinement range is ``2 <= N <= 20``. With the no-argument constructor,
set ``N`` before building the grid. Its initial value is zero (unset). Grid generation validates the current
value, including changes made after construction, and raises an error outside this range.
The planner also raises an error rather than overflowing its fixed storage if a node would have
more than 52 neighbors or a search/path list would have more than 10000 entries.
``NodeList.N`` is the read-only active node count in Python. Use ``append()``, ``pop()``,
and ``clear()`` to change the entries. Python ``append()`` requires a Python-owned node.
The list retains it until its last entry is removed. The search sets each node's predecessor,
``backPointer``, which is read-only in Python.
You can assign a path containing only Python-owned nodes to another planner. A generated path borrows
nodes from its planner's grid, so copying it raises an error. Use ``returnPathCoord()`` to copy its coordinates.
``NodeList.pop()`` and ``NodeList.swap()`` require indices within the active list.
``returnPathCoord()`` returns 1000 for an invalid waypoint index or coordinate axis.

When grid generation clears the previous grid and path, it also resets the endpoint keys
(``keyS`` and ``keyG``). Each component is ``invalidGridIndex`` from construction until an
endpoint pair is selected, and again if a rebuild finds no constraint-free node.
In that case, initialization raises an error before graph search.
This means the sampled grid cannot support planning; it does not prove that no continuous maneuver
exists. Review the constraints or choose another supported refinement level.
Direct calls to ``AStar()`` and ``effortBasedAStar()`` require a successful grid build.
After changing ``N``, call ``GenerateGrid()`` again before running either search directly.
``Reset()`` rebuilds the grid automatically.

Rebuilding the grid or starting a search discards previously prepared waypoints and fitted output.
The planner also discards them when a ``NodeList`` method or assignment to ``path`` changes the node sequence.
After a direct search or path-list edit, call ``pathHandle()`` and then ``spline()`` to fit the new path.
Changing a node's ``sigma_BN`` in place bypasses the list, so call both methods before querying that path.
When editing ``Input`` directly, supply one value per waypoint in each coordinate array and in
any explicit time array. Supply elapsed seconds starting at zero, with finite, strictly increasing times.
Distance-based timing requires distinct consecutive waypoints. A least-squares fit reserves one
control point for each endpoint position and prescribed derivative. The remaining control points
cannot outnumber the interior waypoints. Each prescribed endpoint acceleration requires velocity
at that same endpoint. If weights are supplied, provide one per waypoint. Interior weights
must be finite and nonnegative. A zero interior weight omits that waypoint's residual from
the least-squares objective. The remaining weighted waypoints must still determine all free control points.
``Output.getData()`` and ``Output.getStates()`` raise an error if the fit has been cleared.
``computeTorque()`` and ``computeTorqueNorm()`` require an index in the current fitted samples.
``effortEvaluation()`` requires at least two time samples and one value per time sample in
each attitude, first-derivative, and second-derivative component. The cached ``pathCost``
is NaN until ``Reset()`` accepts a fitted reference.

The start and goal must represent different physical attitudes, including when expressed
in different MRP sets. They must also map to different sampled attitudes. Antipodal
nodes on the MRP unit sphere represent the same sampled attitude. If the endpoints
map to the same sampled attitude, initialization raises an error before graph search.
A finer grid may separate distinct but nearby endpoints.

Even when both endpoints are free, the sampled grid can be disconnected. Both search
methods raise an error if they exhaust the reachable nodes or reach their iteration
limit before reaching the goal. A search that reaches its limit may still have
unexplored nodes. Starting a reset, rebuilding the grid, running either search,
preparing the waypoints, or refitting the spline invalidates the previous reference.
Only a successful reset enables ``UpdateState()`` to write either attitude output.
The maneuver starts at the simulation time passed to that reset. Output messages retain
absolute simulation timestamps.


Detailed Module Description
---------------------------
A detailed explanation of the method implemented in this module can be found in `R. Calaon and H. Schaub, "Constrained Attitude Maneuvering via Modified-Rodrigues-Parameter-Based
Motion Planning Algorithms" <https://arc.aiaa.org/doi/abs/10.2514/1.A35294>`__. To summarize, this module builds a 3D grid in MRP space, whose density is regulated by the input
parameter ``N``. Each grid node corresponds to an attitude. An undirected graph is built with all the nodes that are constraint compliant. Two nodes are added to the graph:

- ``startNode`` whose coordinates :math:`\sigma_{BN,S}` are the spacecraft attitude from ``scStateInMsg``.
- ``goalNode`` whose coordinates :math:`\sigma_{BN,G}` correspond to the desired target attitude at the end of the maneuver.

The distance-based search ranks paths by their accumulated MRP distance plus an estimate of the
distance to the goal. The estimate can exceed the remaining graph distance, so the selected path
can be longer than the shortest grid path. The effort-based search ranks candidate splines by
the integral of the control torque norm. It keeps one predecessor per node even though the effort
depends on the entire path, so another feasible path can have lower effort.

The module fits the selected path using :ref:`BSpline`. When the path contains only the start and goal waypoints,
both fitting modes use a cubic determined by the endpoint attitudes and rates.
When refitting with ``spline()``, prescribing an MRP second derivative at one endpoint
with ``Input.setXDDot_0()`` or ``Input.setXDDot_N()`` gives a quartic. Prescribing both gives a quintic.
Longer paths use a quartic spline. Endpoint angular rates are converted using the MRP
set selected for each waypoint. The planner selects the MRP sequence with the smallest peak norm over the entire short
rotation between each pair of waypoints. At a relative rotation of 180 degrees both
directions are eligible. A tie between starting sets within roundoff favors a principal-set
start. If every sequence reaches an MRP singularity, split the maneuver.

The planner also bounds the total fitted angular travel on each waypoint interval,
including reversals. It accepts a fit only when this bound is less than the short
waypoint separation plus one full revolution. A conservative bound can reject a fit
near this limit. If this occurs, split the maneuver and replan. The effort-based
search skips such candidate paths, as well as candidates with invalid waypoint
intervals or nonfinite control effort. A nonfinite final effort causes reset to raise an error.

Note that this module does not implement the constant angular rate norm routine described in `R. Calaon and H. Schaub <https://arc.aiaa.org/doi/abs/10.2514/1.A35294>`__.
The attitude, rates and accelerations provided to the Attitude Reference Message are those obtained directly from the BSpline interpolation.


User Guide
----------
A maneuver can start at any simulation time. Waypoint times are elapsed seconds from the
planner's most recent successful reset. For example, after a reset at 3600 s, waypoint times
of 0 s and 5 s correspond to simulation times of 3600 s and 3605 s. The attitude reference
messages carry absolute simulation timestamps.

When refitting with ``spline()``, supply finite attitude components at every waypoint,
including zero-weight waypoints. The planner uses each attitude to bound angular travel.
Only the standalone ``BSpline.approximate()`` accepts missing coordinates at zero-weight
interior waypoints, and only when you supply explicit waypoint times.

This example slews from rest at the identity attitude about the inertial z-axis. The body
x-axis must stay outside a 20-degree cone around a fixed celestial direction. Write the
state, inertia, and celestial-body messages before initialization so the planner can read
them at reset. The recorder runs after the planner on the same task.

.. code-block:: python

    import numpy as np
    from Basilisk.architecture import messaging
    from Basilisk.fswAlgorithms import constrainedAttitudeManeuver
    from Basilisk.utilities import SimulationBaseClass, macros

    sim = SimulationBaseClass.SimBaseClass()
    process = sim.CreateNewProcess("process")
    process.addTask(sim.CreateNewTask("guidance", macros.sec2nano(1.0)))  # [s]

    state = messaging.SCStatesMsg().write(messaging.SCStatesMsgPayload())
    vehicle = messaging.VehicleConfigMsgPayload()
    vehicle.ISCPntB_B = [10, 0, 0, 0, 12, 0, 0, 0, 14]  # [kg m^2]
    config = messaging.VehicleConfigMsg().write(vehicle)
    body = messaging.SpicePlanetStateMsgPayload()
    body.PositionVector = [0, 0, 1e8]  # [m] same inertial frame and origin as the spacecraft position
    body_state = messaging.SpicePlanetStateMsg().write(body)

    planner = constrainedAttitudeManeuver.ConstrainedAttitudeManeuver(5)
    planner.sigma_BN_goal = [0, 0, 0.5]  # MRPs [-]
    planner.avgOmega = 0.04  # [rad/s]
    planner.costFcnType = 0  # distance-based search
    planner.BSplineType = 0  # interpolation
    planner.scStateInMsg.subscribeTo(state)
    planner.vehicleConfigInMsg.subscribeTo(config)
    planner.appendKeepOutDirection([1, 0, 0], np.deg2rad(20))  # [-], [rad]
    planner.keepOutCelBodyInMsg.subscribeTo(body_state)

    recorder = planner.attRefOutMsg.recorder()
    sim.AddModelToTask("guidance", planner, ModelPriority=1)
    sim.AddModelToTask("guidance", recorder)
    sim.InitializeSimulation()
    sim.ConfigureStopTime(macros.sec2nano(60.0))  # [s]
    sim.ExecuteSimulation()
    np.testing.assert_allclose(recorder.sigma_RN[-1], planner.sigma_BN_goal, atol=1e-12)

**Cone compliance.** The planner holds each celestial direction fixed at its reset value.
Interpolation passes through the selected attitudes and can cross a cone boundary between
waypoints. Least-squares approximation can also miss an interior waypoint at its assigned time.
Increasing ``N`` changes the sampled path and computational cost. No grid refinement guarantees
continuous clearance. If the maneuver requires it, verify clearance along the full fitted reference
using the expected time-varying celestial directions and a margin for tracking error.

See :ref:`scenarioAttitudeConstrainedManeuver` for a closed-loop spacecraft example with both cone types.

The module is configurable with the following parameters:

.. list-table:: Module Parameters
   :widths: 34 66
   :header-rows: 1

   * - Parameter
     - Description
   * - ``N``
     - grid refinement level, an integer from 2 through 20 inclusive
   * - ``sigma_BN_goal``
     - Required finite goal attitude in MRPs. In Python, assign a sequence of exactly three components. Zero MRPs specify the identity attitude.
   * - ``omega_BN_B_goal``
     - Desired body-frame angular rate at the goal in rad/s. Defaults to zero.
   * - ``avgOmega``
     - Required finite, positive timing parameter in rad/s. The planner sets each waypoint interval to four times the MRP distance between its endpoints divided by ``avgOmega``. The physical mean and peak angular speeds depend on the fitted trajectory.
   * - ``BSplineType``
     - Required before reset. Set to 0 for interpolation or 1 for least-squares approximation. The initial value is -1 (unset).
   * - ``costFcnType``
     - Required before reset. Set to 0 for total MRP distance or 1 for control effort. The initial value is -1 (unset).
