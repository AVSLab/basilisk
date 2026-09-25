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

r"""
Overview
--------

Initialize two Basilisk spacecraft from an Orbit Mean-Elements Message (OMM)
file, then propagate them with point-mass Earth gravity. The bundled
``dataForExamples/ommExample.kvn`` contains **synthetic teaching data**, not
observations or predictions for real satellites. One orbit is nearly circular
and inclined; the other is eccentric and near-polar. Both use fictitious
nine-digit catalog numbers and the same UTC epoch.

Run from the repository root with an installed Basilisk build::

    python examples/scenarioOrbitFromOmm.py

The sample path is resolved relative to this script, so running from another
working directory also works. Keep ``dataForExamples/ommExample.kvn`` alongside
the script when copying the example. No network downloads, SPICE kernels,
gravity coefficient files, or Vizard installation are needed.

Read the file and inspect its metadata
--------------------------------------

:py:func:`Basilisk.utilities.ommHandling.satOmm2elem` detects KVN, XML, JSON,
or CSV from the file contents; the same call handles all four encodings:

.. code-block:: python

    from pathlib import Path
    from Basilisk.utilities import ommHandling

    path = Path(__file__).resolve().parent / "dataForExamples" / "ommExample.kvn"
    records = ommHandling.satOmm2elem(str(path))
    record = records[0]
    print(record.satName, record.noradID, record.ommEpoch)

Catalog IDs are strings, preserving nine-digit IDs and leading zeros.
``ommEpoch`` is a UTC datetime without a timezone object. ``meanMotion`` retains
the input mean motion in revolutions per day. The supported profile is
Earth-centered, TEME, UTC, and SGP4. The sample declares this profile and the
physical units explicitly. Malformed or unsupported records produce a warning
and are skipped, so check the returned list before selecting a satellite.
Unrecoverable file-level errors raise an exception.

Use the converted elements to initialize a spacecraft
-----------------------------------------------------

OMM contains SGP4 **mean** elements. The reader initializes SGP4, evaluates the
state at the record epoch, converts TEME to the Basilisk inertial frame, and
derives **osculating** classical elements in ``record.oe``. Use these elements
for initialization:

.. code-block:: python

    from Basilisk.utilities import orbitalMotion

    r_N, v_N = orbitalMotion.elem2rv(earth.mu, record.oe)
    vehicle.hub.r_CN_NInit = r_N  # [m]
    vehicle.hub.v_CN_NInit = v_N  # [m/s]

Here ``earth`` is the Earth gravity body and ``vehicle`` is a Spacecraft, as
created in :py:func:`run`. The returned semi-major axis is in meters; inclination,
ascending node, argument of periapsis, and true anomaly are in radians.
Eccentricity is dimensionless. Do not apply another degrees-to-radians
conversion, interpret the input mean anomaly as true anomaly, or apply another
TEME rotation to ``record.oe``.

Simulation time zero represents the common OMM epoch. This example rejects
records with different epochs: a catalog's states must first be propagated to
a common epoch before they can describe simultaneous initial conditions.
Static point-mass Earth gravity does not need a separate UTC clock or SPICE
interface; models with time-dependent ephemerides would also need that setup.

Interpret the results
---------------------

The first figure shows the orbital planes, initial positions, and imported
osculating elements. The Earth sphere and the axes share the same length scale.

.. image:: /_images/Scenarios/scenarioOrbitFromOmmOrbits.svg
   :align: center
   :alt: Two imported orbital planes around Earth with their osculating elements.

The second figure covers two periods of the slower spacecraft. The eccentric
spacecraft moves fastest near perigee and slowest near apogee. Dashed altitude
guides come from :math:`a(1-e)-R_E` and :math:`a(1+e)-R_E` using the imported
osculating elements. Altitude here means distance from Earth's center minus
its equatorial radius, not height above an ellipsoid or terrain.

.. image:: /_images/Scenarios/scenarioOrbitFromOmmAltitudeSpeed.svg
   :align: center
   :alt: Altitude and inertial speed histories for circular and eccentric orbits.

SGP4 supplies the initial state only. Subsequent motion in this example uses
Basilisk's two-body dynamics, which omit Earth's oblateness, atmospheric drag,
and other perturbations. The retained ``bStar``, ``nDot``, and ``nDotDot`` fields
do not configure Basilisk force models. These curves therefore illustrate the
handoff to a simulation, rather than continued SGP4 predictions or a validation
of the frame-conversion mathematics.

Try another file
----------------

From Python in the examples directory:

.. code-block:: python

    import scenarioOrbitFromOmm

    results, figures = scenarioOrbitFromOmm.run(False, omm_path="my_catalog.json")
    figures["scenarioOrbitFromOmmOrbits"].savefig("imported-orbits.svg")

Use records at a common epoch with bound orbits that remain above Earth.
``results`` contains the epoch, gravitational parameter, reference Earth
radius, and each satellite's imported record and sampled Cartesian states.
Figures remain available for saving when interactive display is disabled.
"""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import numpy as np

from Basilisk.simulation import spacecraft
from Basilisk.utilities import (
    SimulationBaseClass,
    macros,
    ommHandling,
    orbitalMotion,
    simIncludeGravBody,
)


def run(show_plots, omm_path=None):
    """Import a common-epoch catalog and simulate two periods of its slowest orbit.

    :param show_plots: whether to display the Matplotlib figures
    :param omm_path: optional path to a KVN, XML, JSON, or CSV OMM file
    :return: results dictionary and a dictionary of named Matplotlib figures
    :raises ValueError: if no usable records remain, epochs differ, or an imported
                       orbit is unbound or intersects the reference Earth sphere
    """
    if omm_path is None:
        omm_path = Path(__file__).resolve().parent / "dataForExamples" / "ommExample.kvn"
    records = ommHandling.satOmm2elem(str(omm_path))
    if not records:
        raise ValueError("The OMM file contains no usable records.")
    epoch = records[0].ommEpoch
    if any(record.ommEpoch != epoch for record in records):
        raise ValueError("This example requires all OMM records to share a common epoch.")

    # A static central Earth needs no downloaded ephemeris or gravity data.
    grav_factory = simIncludeGravBody.gravBodyFactory()
    earth = grav_factory.createEarth()
    earth.isCentralBody = True
    for record in records:
        elements = record.oe
        if (not np.all(np.isfinite([elements.a, elements.e]))
                or elements.a <= 0.0 or not 0.0 <= elements.e < 1.0
                or elements.a*(1.0-elements.e) <= earth.radEquator):
            raise ValueError(f"{record.satName} must have a bound osculating orbit above Earth.")

    simulation = SimulationBaseClass.SimBaseClass()
    process = simulation.CreateNewProcess("orbitProcess")
    task_name = "orbitTask"
    step_seconds = 10.0  # [s]
    step_ns = macros.sec2nano(step_seconds)
    process.addTask(simulation.CreateNewTask(task_name, step_ns))
    periods = [2.0*np.pi*np.sqrt(record.oe.a**3/earth.mu) for record in records]  # [s]
    stop_seconds = np.ceil(2.0*max(periods)/step_seconds)*step_seconds  # [s]

    vehicles = []
    recorders = []
    for index, record in enumerate(records):
        vehicle = spacecraft.Spacecraft()
        vehicle.ModelTag = f"ommSatellite{index}"
        vehicle.hub.mHub = 100.0  # [kg]
        vehicle.hub.IHubPntBc_B = np.diag([10.0, 10.0, 10.0]).tolist()  # [kg m^2]
        grav_factory.addBodiesTo(vehicle)

        # oe is already osculating and inertial, with a in meters and angles in radians.
        r_N, v_N = orbitalMotion.elem2rv(earth.mu, record.oe)
        vehicle.hub.r_CN_NInit = r_N  # [m]
        vehicle.hub.v_CN_NInit = v_N  # [m/s]
        simulation.AddModelToTask(task_name, vehicle)
        recorder = vehicle.scStateOutMsg.recorder(step_ns)
        simulation.AddModelToTask(task_name, recorder)
        vehicles.append(vehicle)
        recorders.append(recorder)

    simulation.InitializeSimulation()
    simulation.ConfigureStopTime(macros.sec2nano(stop_seconds))
    simulation.ExecuteSimulation()

    satellites = []
    for record, recorder, period in zip(records, recorders, periods):
        satellites.append({
            "record": record,
            "time_s": np.array(recorder.times())*macros.NANO2SEC,
            "position_m": np.array(recorder.r_BN_N),
            "velocity_m_s": np.array(recorder.v_BN_N),
            "period_s": period,
        })
    results = {"epoch": epoch, "mu": earth.mu, "earth_radius_m": earth.radEquator,
               "satellites": satellites}
    figures = _plot_results(results)
    if show_plots:
        plt.show()
    # Close only this scenario's figures; callers can still save the returned objects.
    for figure in figures.values():
        plt.close(figure)
    return results, figures


def _plot_results(results):
    """Plot orbit geometry and two-body altitude/speed histories with consistent colors."""
    km_per_m = 1e-3  # [km/m]
    seconds_per_minute = 60.0  # [s/min]
    radius_km = results["earth_radius_m"]*km_per_m  # [km]
    satellites = results["satellites"]
    colors = [plt.get_cmap("tab10")(index % 10) for index in range(len(satellites))]
    epoch_label = results["epoch"].isoformat(sep=" ") + " UTC"

    with plt.rc_context({"font.size": 11, "axes.spines.top": False,
                         "axes.spines.right": False, "svg.fonttype": "none"}):
        orbit_figure = plt.figure(figsize=(12, max(6.5, 2.6*len(satellites))), layout="constrained")
        grid = orbit_figure.add_gridspec(1, 2, width_ratios=[1.8, 1])
        orbit_axes = orbit_figure.add_subplot(grid[0], projection="3d")
        details_axes = orbit_figure.add_subplot(grid[1])
        details_axes.axis("off")
        orbit_figure.suptitle("From OMM to Basilisk: imported orbits", fontsize=17)
        orbit_axes.set_title("Inertial frame | dots mark the record epoch", fontsize=11)
        longitude = np.linspace(0.0, 2.0*np.pi, 40)  # [rad]
        colatitude = np.linspace(0.0, np.pi, 20)  # [rad]
        orbit_axes.plot_surface(
            radius_km*np.outer(np.cos(longitude), np.sin(colatitude)),
            radius_km*np.outer(np.sin(longitude), np.sin(colatitude)),
            radius_km*np.outer(np.ones_like(longitude), np.cos(colatitude)),
            color="#abc7da", alpha=0.3, linewidth=0, shade=False,
        )
        orbit_axes.plot(radius_km*np.cos(longitude), radius_km*np.sin(longitude),
                        np.zeros_like(longitude), color="#677f8e", linewidth=0.8)
        max_radius = radius_km
        for index, (satellite, color) in enumerate(zip(satellites, colors)):
            record = satellite["record"]
            position_km = satellite["position_m"]*km_per_m  # [km]
            first_orbit = satellite["time_s"] <= satellite["period_s"]
            orbit_axes.plot(*position_km[first_orbit].T, color=color, linewidth=2)
            orbit_axes.scatter(*position_km[0], color=color, s=45, edgecolors="white", depthshade=False)
            max_radius = max(max_radius, np.max(np.linalg.norm(position_km, axis=1)))
            elements = record.oe
            detail = (f"{record.satName}\nCatalog ID: {record.noradID}\n"
                      f"Input mean motion: {record.meanMotion:g} rev/day\n"
                      f"a = {elements.a*km_per_m:,.1f} km   e = {elements.e:.4f}\n"
                      f"i = {np.degrees(elements.i):.2f} deg\n"
                      f"Two-body period: {satellite['period_s']/seconds_per_minute:.1f} min")
            details_axes.text(0.02, 0.88-index*0.76/len(satellites), detail,
                              transform=details_axes.transAxes, va="top", linespacing=1.7,
                              color=color, bbox={"facecolor": "#f5f7fa", "edgecolor": color,
                                                 "boxstyle": "round,pad=0.8"})
        details_axes.text(0.02, 0.04, f"Osculating elements at\n{epoch_label}\nPoint-mass Earth propagation",
                          transform=details_axes.transAxes, fontsize=10, linespacing=1.6)
        limit = 1.08*max_radius  # [km]
        orbit_axes.set(xlim=(-limit, limit), ylim=(-limit, limit), zlim=(-limit, limit),
                       xlabel="Inertial x [km]", ylabel="Inertial y [km]", zlabel="Inertial z [km]")
        orbit_axes.set_box_aspect((1, 1, 1))
        orbit_axes.view_init(elev=45, azim=30)  # [deg] view both orbital planes obliquely
        for axis in (orbit_axes.xaxis, orbit_axes.yaxis, orbit_axes.zaxis):
            axis.set_major_locator(MaxNLocator(nbins=4, steps=[1, 2, 5, 10], prune="both"))
            axis.pane.fill = False

        history_figure, history_axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True, layout="constrained")
        history_figure.suptitle("After import: Basilisk two-body propagation", fontsize=17)
        history_axes[0].set_title(
            f"Simulation time zero = {epoch_label}\nDashed altitude guides: imported perigee / apogee",
            fontsize=10,
        )
        for satellite, color in zip(satellites, colors):
            record = satellite["record"]
            time_minutes = satellite["time_s"]/seconds_per_minute  # [min]
            altitude_km = np.linalg.norm(satellite["position_m"], axis=1)*km_per_m-radius_km  # [km]
            speed_km_s = np.linalg.norm(satellite["velocity_m_s"], axis=1)*km_per_m  # [km/s]
            history_axes[0].plot(time_minutes, altitude_km, color=color, label=record.satName, linewidth=2)
            history_axes[1].plot(time_minutes, speed_km_s, color=color, linewidth=2)
            for sign in (-1, 1):
                apsis_km = record.oe.a*(1.0+sign*record.oe.e)*km_per_m-radius_km  # [km]
                history_axes[0].axhline(apsis_km, color=color, linestyle="--", linewidth=0.9, alpha=0.5)
        history_axes[0].set_ylabel("Spherical altitude [km]")
        history_axes[0].legend(loc="upper right", fontsize=10)
        history_axes[1].set(ylabel="Inertial speed [km/s]", xlabel="Time since OMM epoch [min]")
        for axes in history_axes:
            axes.grid(alpha=0.2)
            axes.margins(x=0)

    return {"scenarioOrbitFromOmmOrbits": orbit_figure,
            "scenarioOrbitFromOmmAltitudeSpeed": history_figure}


if __name__ == "__main__":
    run(True)
