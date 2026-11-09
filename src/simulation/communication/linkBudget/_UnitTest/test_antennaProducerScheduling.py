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

"""Exercise antenna initialization through the simulation scheduler.

Real spacecraft, ground, antenna, link-budget, downlink and storage modules run
on ordered tasks. Delaying either the ground or antenna task exercises the
waiting state over multiple timestamps. Reinitializing the same simulation
after an altitude change exercises previously written producer messages.
"""

import numpy as np
import pytest

from Basilisk.architecture import messaging
from Basilisk.simulation import (
    downlinkHandling,
    groundLocation,
    linkBudget,
    simpleAntenna,
    simpleStorageUnit,
    spacecraft,
)
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.utilities import SimulationBaseClass, macros, orbitalMotion


TASK_PERIOD = 1.0  # [s]
FREQUENCY = 20.0e9  # [Hz]
BANDWIDTH = 8.0e6  # [Hz]
INITIAL_ALTITUDE = 1624.0  # [m]
ALTITUDE_CHANGE = 2000.0  # [m]
LINK_DISTANCE = 2000.0e3  # [m]
INITIAL_DATA = 10000000  # [bit]


def configure_antenna(antenna, name, state):
    """Configure an antenna without sky-map or ephemeris dependencies.

    :param antenna: Antenna module to configure.
    :param name: Published antenna identifier.
    :param state: Transmit or receive antenna state.
    """
    antenna.ModelTag = name
    antenna.setAntennaName(name)
    antenna.setAntennaState(state)
    antenna.setAntennaFrequency(FREQUENCY)
    antenna.setAntennaBandwidth(BANDWIDTH)
    antenna.setAntennaDirectivity_dB(30.0)  # [dBi]
    antenna.setAntennaHpbwRatio(1.0)  # [-]
    antenna.setAntennaP_Tx(5.0)  # [W]
    antenna.setAntennaP_Rx(5.0)  # [W]
    antenna.setAntennaEquivalentNoiseTemp(120.0)  # [K]
    antenna.setAntennaRadEfficiency(0.65)  # [-]
    antenna.setAntennaEnvironmentTemperature(150.0)  # [K]


class ScheduledLink:
    """Keep the scheduled producer chain and its recorders alive together."""

    def __init__(self, ground_first):
        """Connect the chain with explicit task and model priorities.

        :param ground_first: Whether the ground antenna uses link input one.
        """
        self.sim = SimulationBaseClass.SimBaseClass()
        process = self.sim.CreateNewProcess("communication")
        for name, priority in (
            ("spacecraft", 40), ("ground", 30), ("antennas", 20), ("link", 10)
        ):
            task = self.sim.CreateNewTask(name, macros.sec2nano(TASK_PERIOD))
            process.addTask(task, priority)

        self.spacecraft = spacecraft.Spacecraft()
        self.spacecraft.ModelTag = "spacecraft"
        self.ground = groundLocation.GroundLocation()
        self.ground.ModelTag = "ground"
        self.ground.planetRadius = orbitalMotion.REQ_EARTH*1.0e3  # [m]
        self.ground.addSpacecraftToModel(self.spacecraft.scStateOutMsg)
        self.transmitter = simpleAntenna.SimpleAntenna()
        self.receiver = simpleAntenna.SimpleAntenna()
        configure_antenna(self.transmitter, "space", simpleAntenna.ANTENNA_TX)
        configure_antenna(self.receiver, "ground", simpleAntenna.ANTENNA_RX)
        self.transmitter.scStateInMsg.subscribeTo(self.spacecraft.scStateOutMsg)
        self.receiver.groundStateInMsg.subscribeTo(self.ground.currentGroundStateOutMsg)

        self.budget = linkBudget.LinkBudget()
        self.budget.ModelTag = "linkBudget"
        self.budget.atmosAtt = True
        antennas = (self.receiver, self.transmitter) if ground_first else (self.transmitter, self.receiver)
        self.budget.antennaInPayload_1.subscribeTo(antennas[0].antennaOutMsg)
        self.budget.antennaInPayload_2.subscribeTo(antennas[1].antennaOutMsg)
        self.downlink = downlinkHandling.DownlinkHandling()
        self.downlink.ModelTag = "downlink"
        self.downlink.setBitRateRequest(100000.0)  # [bit/s]
        self.downlink.linkBudgetInMsg.subscribeTo(self.budget.linkBudgetOutPayload)
        self.storage = simpleStorageUnit.SimpleStorageUnit()
        self.storage.ModelTag = "storage"
        self.storage.storageCapacity = 2*INITIAL_DATA
        self.storage.setDataBuffer(INITIAL_DATA)
        self.storage.addDataNodeToModel(self.downlink.nodeDataOutMsg)
        self.downlink.addStorageUnitToDownlink(self.storage.storageUnitDataOutMsg)

        for task, model, priority in (
            ("spacecraft", self.spacecraft, 10),
            ("ground", self.ground, 10),
            ("antennas", self.transmitter, 20),
            ("antennas", self.receiver, 10),
            ("link", self.budget, 30),
            ("link", self.downlink, 20),
            ("link", self.storage, 10),
        ):
            self.sim.AddModelToTask(task, model, ModelPriority=priority)
        self.link_log = self.budget.linkBudgetOutPayload.recorder()
        self.antenna_log = self.receiver.antennaOutMsg.recorder()
        self.downlink_log = self.downlink.downlinkOutMsg.recorder()
        self.node_log = self.downlink.nodeDataOutMsg.recorder()
        self.storage_log = self.storage.storageUnitDataOutMsg.recorder()
        self.loss_log = self.budget.logger(["L_atm", "L_point"])
        for recorder in (
            self.link_log, self.antenna_log, self.downlink_log,
            self.node_log, self.storage_log, self.loss_log,
        ):
            self.sim.AddModelToTask("link", recorder, ModelPriority=0)
        self.set_geometry(INITIAL_ALTITUDE)

    def set_geometry(self, altitude):
        """Set stationary geometry with the spacecraft boresight at the station.

        :param altitude: Ground station altitude in meters.
        """
        latitude = 0.35  # [rad]
        longitude = -0.6  # [rad]
        elevation = 0.7  # [rad]
        self.ground.specifyLocation(latitude, longitude, altitude)
        self.normal = np.array([
            np.cos(latitude)*np.cos(longitude),
            np.cos(latitude)*np.sin(longitude), np.sin(latitude),
        ])  # [-]
        self.ground_position = (self.ground.planetRadius + altitude)*self.normal
        east = np.array([-np.sin(longitude), np.cos(longitude), 0.0])  # [-]
        ray = np.sin(elevation)*self.normal + np.cos(elevation)*east
        self.spacecraft_position = self.ground_position + LINK_DISTANCE*ray
        self.spacecraft.hub.r_CN_NInit = self.spacecraft_position
        boresight = -ray
        axis_y = np.cross(boresight, self.normal)
        axis_y /= np.linalg.norm(axis_y)
        axis_x = np.cross(axis_y, boresight)
        self.spacecraft.hub.sigma_BNInit = rbk.C2MRP(np.array([axis_x, axis_y, boresight]))

    def run_until(self, time_seconds):
        """Advance the scheduler without manually updating any producer.

        :param time_seconds: Absolute stop time in seconds.
        """
        self.sim.ConfigureStopTime(macros.sec2nano(time_seconds))
        self.sim.ExecuteSimulation()

    def assert_reset_output(self):
        """Check that reset publishes zero service even before the first tick."""
        assert self.downlink.linkBudgetInMsg.isWritten()
        output = self.budget.linkBudgetOutPayload.read()
        assert (output.CNR1, output.CNR2, output.bandwidth) == (0.0, 0.0, 0.0)
        assert self.budget.getL_atm() == 0.0

    def assert_ready(self, first_sample, ground_first):
        """Compare scheduled results with an explicitly primed link reference.

        The reference uses published RF properties but independently supplied
        positions and surface normal. Existing unit tests validate the RF and
        atmospheric formulas; this test validates their scheduled initialization.

        :param first_sample: First recorder index with ready producer inputs.
        :param ground_first: Whether the ground antenna uses link input one.
        :return: Reference atmospheric attenuation in decibels.
        """
        ground_payload = self.receiver.antennaOutMsg.read()
        ground_payload.r_AN_N = self.ground_position
        ground_payload.r_AP_N = self.ground_position
        ground_payload.nHat_LP_N = self.normal
        space_payload = self.transmitter.antennaOutMsg.read()
        space_payload.r_AN_N = self.spacecraft_position
        ground_msg = messaging.AntennaLogMsg().write(ground_payload)
        space_msg = messaging.AntennaLogMsg().write(space_payload)
        reference = linkBudget.LinkBudget()
        reference.atmosAtt = True
        reference.antennaInPayload_1.subscribeTo(space_msg)
        reference.antennaInPayload_2.subscribeTo(ground_msg)
        reference.Reset(0)  # [ns]
        reference.UpdateState(0)  # [ns]
        expected_cnr = reference.getCNR2()
        expected_loss = reference.getL_atm()  # [dB]
        assert expected_cnr > 0.0 and expected_loss > 0.0
        assert self.budget.atmosAtt
        ready = slice(first_sample, None)
        cnr = self.link_log.CNR1 if ground_first else self.link_log.CNR2
        unused_cnr = self.link_log.CNR2 if ground_first else self.link_log.CNR1
        np.testing.assert_allclose(cnr[ready], expected_cnr, rtol=1.0e-10)
        np.testing.assert_array_equal(unused_cnr, 0.0)
        np.testing.assert_allclose(
            self.loss_log.L_atm[ready], expected_loss, atol=1.0e-12, rtol=0.0
        )  # [dB]
        np.testing.assert_allclose(self.loss_log.L_point[ready], 0.0, atol=1.0e-12)  # [dB]
        # Match geometry samples to the timestamps at which the link is ready.
        antenna_ready = self.antenna_log.times() >= self.link_log.times()[first_sample]
        np.testing.assert_array_equal(self.antenna_log.times()[antenna_ready], self.link_log.times()[ready])
        np.testing.assert_allclose(
            self.antenna_log.nHat_LP_N[antenna_ready] - self.normal, 0.0, atol=1.0e-14
        )
        np.testing.assert_allclose(
            self.antenna_log.r_AP_N[antenna_ready] - self.ground_position, 0.0, atol=1.0e-8
        )  # [m]
        np.testing.assert_array_equal(self.link_log.bandwidth[ready], BANDWIDTH)
        np.testing.assert_array_equal(self.downlink_log.linkActive[ready], 1)
        np.testing.assert_array_equal(self.downlink_log.receiverIndex[ready], 1 if ground_first else 2)
        np.testing.assert_allclose(self.downlink_log.cnr[ready], expected_cnr, rtol=1.0e-10)
        np.testing.assert_array_equal(self.link_log.timesWritten(), self.link_log.times())
        np.testing.assert_array_equal(self.downlink_log.timesWritten(), self.link_log.times())
        np.testing.assert_allclose(self.node_log.baudRate, -self.downlink_log.storageRemovalRate)
        elapsed = np.diff(self.link_log.times(), prepend=0)*macros.NANO2SEC  # [s]
        np.testing.assert_allclose(self.downlink_log.timeStep, elapsed)
        # At time zero the elapsed interval is zero; later ready ticks must deliver data immediately.
        flow_start = max(first_sample, 1)
        assert np.all(self.downlink_log.deliveredDataRate[flow_start:] > 0.0)
        assert np.all(self.node_log.baudRate[flow_start:] < 0.0)
        assert self.storage_log.storageLevel[-1] < self.storage_log.storageLevel[0]
        removed = np.sum(self.downlink_log.storageRemovalRate*self.downlink_log.timeStep)  # [bit]
        assert self.downlink_log.cumulativeRemovedBits[-1] == pytest.approx(removed)
        assert self.storage_log.storageLevel[0] - self.storage_log.storageLevel[-1] == pytest.approx(
            removed, abs=len(self.storage_log.times())
        )  # [bit] Storage rounds each integrated update to an integer number of bits.
        return expected_loss


@pytest.mark.parametrize("ground_first", [False, True], ids=["ground-second", "ground-first"])
@pytest.mark.parametrize(
    "delayed_task", [None, "antennas", "ground"],
    ids=["ordered", "delayed-antennas", "delayed-ground"],
)
def test_scheduled_antenna_initialization(ground_first, delayed_task):
    """Validate initialization, waiting, downstream service and reset scheduling.

    Validation Test Description
    ---------------------------
    Delayed tasks remain disabled for two ticks while consumers keep running,
    then resume in producer order. A full simulation reset must subsequently
    rebuild the atmosphere at the changed altitude on its first scheduled tick.

    Test Parameter Discussion
    -------------------------
    Both link input orders are exercised with normal startup, delayed antenna
    publication, and antennas publishing before the ground position is ready.

    Description of Variables Being Tested
    -------------------------------------
    Check CNR, atmospheric and pointing loss, ground normal and position,
    message timestamps, downlink service and integrated storage removal.

    :param ground_first: Whether the ground antenna uses link input one.
    :param delayed_task: Producer task to delay, or ``None`` for normal startup.
    """
    chain = ScheduledLink(ground_first)
    chain.sim.InitializeSimulation()
    assert not chain.budget.antennaInPayload_1.isWritten()
    assert not chain.budget.antennaInPayload_2.isWritten()
    chain.assert_reset_output()
    if delayed_task is not None:
        chain.sim.disableTask(delayed_task)
    chain.run_until(TASK_PERIOD)
    if delayed_task is not None:
        for values in (
            chain.link_log.CNR1, chain.link_log.CNR2, chain.link_log.bandwidth,
            chain.loss_log.L_atm, chain.downlink_log.linkActive,
            chain.downlink_log.deliveredDataRate, chain.downlink_log.cumulativeRemovedBits,
            chain.node_log.baudRate,
        ):
            np.testing.assert_array_equal(values, 0.0)
        np.testing.assert_array_equal(chain.storage_log.storageLevel, INITIAL_DATA)
        chain.sim.enableTask(delayed_task)
    chain.run_until(3*TASK_PERIOD)
    expected_times = np.arange(4)*macros.sec2nano(TASK_PERIOD)  # [ns]
    np.testing.assert_array_equal(chain.link_log.times(), expected_times)
    first_loss = chain.assert_ready(0 if delayed_task is None else 2, ground_first)

    previous_position = chain.ground_position.copy()
    chain.set_geometry(INITIAL_ALTITUDE + ALTITUDE_CHANGE)
    chain.sim.InitializeSimulation()
    chain.assert_reset_output()
    # The old antenna publication survives reset; the next ordered tick must replace it.
    np.testing.assert_allclose(chain.receiver.antennaOutMsg.read().r_AP_N, previous_position)
    chain.run_until(2*TASK_PERIOD)
    np.testing.assert_array_equal(chain.link_log.times(), expected_times[:3])
    assert chain.assert_ready(0, ground_first) < first_loss


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__]))
